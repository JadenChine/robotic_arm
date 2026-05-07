import Combine
import CoreBluetooth
import Foundation

enum ArmControlMode: UInt8, Equatable {
    case joint = 0
    case cartesian = 1

    var displayName: String {
        switch self {
        case .joint:
            return "Joint"
        case .cartesian:
            return "Cartesian"
        }
    }
}

final class BLEManager: NSObject, ObservableObject {
    @Published var discoveredPeripherals: [CBPeripheral] = []
    @Published var connectionState: String = "Bluetooth not ready"
    @Published var activePeripheralName: String = "Not connected"
    @Published var isConnected: Bool = false
    @Published var hasControl: Bool = false
    @Published var lastError: String = ""
    @Published var controlMode: ArmControlMode = .joint

    private var central: CBCentralManager?
    private var connectedPeripheral: CBPeripheral?

    private var controlPointCharacteristic: CBCharacteristic?
    private var ownerStatusCharacteristic: CBCharacteristic?
    private var jointTargetsCharacteristic: CBCharacteristic?
    private var cartesianTargetsCharacteristic: CBCharacteristic?

    private var pendingJointTargetsDegrees: [Double] = Array(repeating: 0.0, count: 6)
    private var pendingCartesianTargetsMM: [Double] = [0.0, 0.0, 429.0]
    private var pendingJointTargetsDirty = false
    private var pendingCartesianTargetsDirty = false
    private var disconnectAfterNextWrite: Bool = false
    private var jointWriteInFlight: Bool = false
    private var cartesianWriteInFlight: Bool = false

    private var targetSendTimer: Timer?
    private var ownerPollTimer: Timer?

    private enum LastControlCommand: Equatable {
        case none
        case take
        case release
        case releaseAndDisconnect
        case setMode(ArmControlMode)
    }

    private var lastControlCommand: LastControlCommand = .none

    private let armServiceUUID = CBUUID(string: "FFF0")
    private let controlPointUUID = CBUUID(string: "FFF1")
    private let ownerTokenUUID = CBUUID(string: "FFF2")
    private let jointTargetsUUID = CBUUID(string: "FFF3")
    private let cartesianTargetsUUID = CBUUID(string: "FFF4")

    private let jointLimitsDeciDegrees: [(min: Int, max: Int)] = [
        (min: -1200, max: 100),
        (min: -1200, max: 1200),
        (min: -1200, max: 1200),
        (min: -1200, max: 1200),
        (min: -900, max: 900),
        (min: -1200, max: 1200),
    ]

    private let cartesianLimitsMM: [(min: Int, max: Int)] = [
        (min: -364, max: 364),
        (min: -364, max: 364),
        (min: 0, max: 429),
    ]

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: .main)

        let targetTimer = Timer(timeInterval: 1.0 / 60.0, repeats: true) { [weak self] _ in
            self?.flushPendingTargetsIfNeeded()
        }
        RunLoop.main.add(targetTimer, forMode: .common)
        targetSendTimer = targetTimer

        let ownerTimer = Timer(timeInterval: 0.5, repeats: true) { [weak self] _ in
            self?.pollOwnerStatusIfPossible()
        }
        RunLoop.main.add(ownerTimer, forMode: .common)
        ownerPollTimer = ownerTimer
    }

    deinit {
        targetSendTimer?.invalidate()
        ownerPollTimer?.invalidate()
    }

    func startScan() {
        guard let central else { return }
        guard central.state == .poweredOn else {
            connectionState = bluetoothStateMessage(for: central.state)
            return
        }

        discoveredPeripherals.removeAll()
        connectionState = "Scanning for arm peripheral..."
        central.scanForPeripherals(withServices: [armServiceUUID], options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
    }

    func stopScan() {
        central?.stopScan()
        if connectedPeripheral == nil {
            connectionState = "Scan stopped"
        }
    }

    func connect(to peripheral: CBPeripheral) {
        guard let central else { return }

        stopScan()
        connectionState = "Connecting to \(peripheral.name ?? "Unnamed")..."
        central.connect(peripheral, options: nil)
    }

    func disconnect() {
        guard let peripheral = connectedPeripheral else { return }

        // Best-effort: try to release before disconnecting.
        disconnectAfterNextWrite = true
        if let characteristic = controlPointCharacteristic {
            let payload = Data([0x02])
            lastControlCommand = .releaseAndDisconnect
            peripheral.writeValue(payload, for: characteristic, type: .withResponse)

            // Fallback: if the write never completes, still disconnect.
            DispatchQueue.main.asyncAfter(deadline: .now() + 0.8) { [weak self] in
                guard let self else { return }
                guard self.disconnectAfterNextWrite else { return }
                guard let connectedPeripheral = self.connectedPeripheral else { return }
                self.central?.cancelPeripheralConnection(connectedPeripheral)
                self.disconnectAfterNextWrite = false
                self.lastControlCommand = .none
            }
        } else {
            central?.cancelPeripheralConnection(peripheral)
        }
    }

    func takeControl() {
        guard let peripheral = connectedPeripheral,
              let characteristic = controlPointCharacteristic else {
            lastError = "Not connected to control characteristic"
            return
        }

        let payload = Data([0x01])
        lastControlCommand = .take
        peripheral.writeValue(payload, for: characteristic, type: .withResponse)
    }

    func releaseControl() {
        guard let peripheral = connectedPeripheral,
              let characteristic = controlPointCharacteristic else {
            lastError = "Not connected to control characteristic"
            return
        }

        let payload = Data([0x02])
        lastControlCommand = .release
        peripheral.writeValue(payload, for: characteristic, type: .withResponse)
    }

    func setControlMode(_ mode: ArmControlMode) {
        controlMode = mode
        guard hasControl else { return }
        writeControlModeNow(mode)
    }

    func updateJointTargetsDegrees(_ jointAngles: [Double]) {
        guard jointAngles.count == 6 else {
            lastError = "Joint array must contain exactly 6 values"
            return
        }

        pendingJointTargetsDegrees = jointAngles
        pendingJointTargetsDirty = true
    }

    func updateCartesianTargetsMM(_ cartesianTargets: [Double]) {
        guard cartesianTargets.count == 3 else {
            lastError = "Cartesian target array must contain exactly 3 values"
            return
        }

        pendingCartesianTargetsMM = cartesianTargets
        pendingCartesianTargetsDirty = true
    }

    private func writeControlModeNow(_ mode: ArmControlMode) {
        guard let peripheral = connectedPeripheral,
              let characteristic = controlPointCharacteristic else {
            return
        }

        let payload = Data([0x03, mode.rawValue])
        lastControlCommand = .setMode(mode)
        peripheral.writeValue(payload, for: characteristic, type: .withResponse)
    }

    private func flushPendingTargetsIfNeeded() {
        switch controlMode {
        case .joint:
            flushPendingJointTargetsIfNeeded()
        case .cartesian:
            flushPendingCartesianTargetsIfNeeded()
            flushPendingJointTargetsIfNeeded()
        }
    }

    private func flushPendingJointTargetsIfNeeded() {
        guard pendingJointTargetsDirty else { return }
        guard hasControl else { return }

        guard let peripheral = connectedPeripheral,
              let characteristic = jointTargetsCharacteristic else {
            return
        }

        let packet = encodeJointTargetsPacket(pendingJointTargetsDegrees)
        if characteristic.properties.contains(.writeWithoutResponse) {
            guard peripheral.canSendWriteWithoutResponse else { return }
            peripheral.writeValue(packet, for: characteristic, type: .withoutResponse)
        } else {
            guard !jointWriteInFlight else { return }
            jointWriteInFlight = true
            peripheral.writeValue(packet, for: characteristic, type: .withResponse)
        }

        pendingJointTargetsDirty = false
    }

    private func flushPendingCartesianTargetsIfNeeded() {
        guard pendingCartesianTargetsDirty else { return }
        guard hasControl else { return }

        guard let peripheral = connectedPeripheral,
              let characteristic = cartesianTargetsCharacteristic else {
            return
        }

        let packet = encodeCartesianTargetsPacket(pendingCartesianTargetsMM)
        if characteristic.properties.contains(.writeWithoutResponse) {
            guard peripheral.canSendWriteWithoutResponse else { return }
            peripheral.writeValue(packet, for: characteristic, type: .withoutResponse)
        } else {
            guard !cartesianWriteInFlight else { return }
            cartesianWriteInFlight = true
            peripheral.writeValue(packet, for: characteristic, type: .withResponse)
        }

        pendingCartesianTargetsDirty = false
    }

    private func flushActiveTargetsNow() {
        switch controlMode {
        case .joint:
            flushJointTargetsNow()
        case .cartesian:
            flushCartesianTargetsNow()
            flushJointTargetsNow()
        }
    }

    private func flushJointTargetsNow() {
        guard let peripheral = connectedPeripheral,
              let characteristic = jointTargetsCharacteristic else {
            return
        }

        let packet = encodeJointTargetsPacket(pendingJointTargetsDegrees)
        if characteristic.properties.contains(.writeWithoutResponse) {
            guard peripheral.canSendWriteWithoutResponse else {
                pendingJointTargetsDirty = true
                return
            }
            peripheral.writeValue(packet, for: characteristic, type: .withoutResponse)
        } else {
            guard !jointWriteInFlight else {
                pendingJointTargetsDirty = true
                return
            }
            jointWriteInFlight = true
            peripheral.writeValue(packet, for: characteristic, type: .withResponse)
        }

        pendingJointTargetsDirty = false
    }

    private func flushCartesianTargetsNow() {
        guard let peripheral = connectedPeripheral,
              let characteristic = cartesianTargetsCharacteristic else {
            return
        }

        let packet = encodeCartesianTargetsPacket(pendingCartesianTargetsMM)
        if characteristic.properties.contains(.writeWithoutResponse) {
            guard peripheral.canSendWriteWithoutResponse else {
                pendingCartesianTargetsDirty = true
                return
            }
            peripheral.writeValue(packet, for: characteristic, type: .withoutResponse)
        } else {
            guard !cartesianWriteInFlight else {
                pendingCartesianTargetsDirty = true
                return
            }
            cartesianWriteInFlight = true
            peripheral.writeValue(packet, for: characteristic, type: .withResponse)
        }

        pendingCartesianTargetsDirty = false
    }

    private func encodeJointTargetsPacket(_ jointAngles: [Double]) -> Data {
        var data = Data(capacity: 12)

        for (index, angle) in jointAngles.enumerated() {
            let limits = jointLimitsDeciDegrees[index]
            let deciDegreesInt = max(limits.min, min(limits.max, Int(round(angle * 10.0))))
            let deciDegrees = Int16(deciDegreesInt)
            var littleEndianValue = deciDegrees.littleEndian
            withUnsafeBytes(of: &littleEndianValue) { bytes in
                data.append(contentsOf: bytes)
            }
        }

        return data
    }

    private func encodeCartesianTargetsPacket(_ cartesianTargets: [Double]) -> Data {
        var data = Data(capacity: 6)

        for (index, target) in cartesianTargets.enumerated() {
            let limits = cartesianLimitsMM[index]
            let targetInt = max(limits.min, min(limits.max, Int(round(target))))
            let targetMM = Int16(targetInt)
            var littleEndianValue = targetMM.littleEndian
            withUnsafeBytes(of: &littleEndianValue) { bytes in
                data.append(contentsOf: bytes)
            }
        }

        return data
    }

    private func pollOwnerStatusIfPossible() {
        guard connectedPeripheral != nil else { return }
        readOwnerStatus()
    }

    private func readOwnerStatus() {
        guard let peripheral = connectedPeripheral,
              let characteristic = ownerStatusCharacteristic else {
            return
        }

        peripheral.readValue(for: characteristic)
    }

    private func resetConnectionState() {
        connectedPeripheral = nil
        controlPointCharacteristic = nil
        ownerStatusCharacteristic = nil
        jointTargetsCharacteristic = nil
        cartesianTargetsCharacteristic = nil
        activePeripheralName = "Not connected"
        hasControl = false
        lastControlCommand = .none
        pendingJointTargetsDirty = false
        pendingCartesianTargetsDirty = false
        jointWriteInFlight = false
        cartesianWriteInFlight = false
        isConnected = false
        disconnectAfterNextWrite = false
    }
}

extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            connectionState = "Bluetooth ready"
            startScan()
        case .poweredOff:
            connectionState = bluetoothStateMessage(for: .poweredOff)
            resetConnectionState()
        case .resetting:
            connectionState = bluetoothStateMessage(for: .resetting)
            resetConnectionState()
        case .unauthorized:
            connectionState = bluetoothStateMessage(for: .unauthorized)
            resetConnectionState()
        case .unsupported:
            connectionState = bluetoothStateMessage(for: .unsupported)
            resetConnectionState()
        case .unknown:
            fallthrough
        @unknown default:
            connectionState = bluetoothStateMessage(for: .unknown)
            resetConnectionState()
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any],
                        rssi RSSI: NSNumber) {
        _ = advertisementData
        _ = RSSI

        if discoveredPeripherals.contains(where: { $0.identifier == peripheral.identifier }) {
            return
        }

        discoveredPeripherals.append(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        connectedPeripheral = peripheral
        peripheral.delegate = self
        activePeripheralName = peripheral.name ?? "Unnamed"
        isConnected = true
        connectionState = "Connected, discovering services..."
        peripheral.discoverServices([armServiceUUID])
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral,
                        error: Error?) {
        _ = peripheral
        connectionState = "Connection failed"
        if let error {
            lastError = error.localizedDescription
        }
        resetConnectionState()
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        _ = peripheral
        connectionState = "Disconnected"
        if let error {
            lastError = error.localizedDescription
        }
        resetConnectionState()

        // Return to a clean scanning state automatically.
        startScan()
    }
}

extension BLEManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverServices error: Error?) {
        if let error {
            connectionState = "Service discovery failed"
            lastError = error.localizedDescription
            return
        }

        guard let services = peripheral.services else {
            connectionState = "No services discovered"
            return
        }

        for service in services where service.uuid == armServiceUUID {
            peripheral.discoverCharacteristics(
                [controlPointUUID, ownerTokenUUID, jointTargetsUUID, cartesianTargetsUUID],
                for: service
            )
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService,
                    error: Error?) {
        if let error {
            connectionState = "Characteristic discovery failed"
            lastError = error.localizedDescription
            return
        }

        guard let characteristics = service.characteristics else { return }

        for characteristic in characteristics {
            switch characteristic.uuid {
            case controlPointUUID:
                controlPointCharacteristic = characteristic
            case ownerTokenUUID:
                ownerStatusCharacteristic = characteristic
            case jointTargetsUUID:
                jointTargetsCharacteristic = characteristic
            case cartesianTargetsUUID:
                cartesianTargetsCharacteristic = characteristic
            default:
                break
            }
        }

        if controlPointCharacteristic != nil,
           ownerStatusCharacteristic != nil,
           jointTargetsCharacteristic != nil,
           cartesianTargetsCharacteristic != nil {
            connectionState = "Connected and ready"
            readOwnerStatus()
            flushPendingTargetsIfNeeded()
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        _ = peripheral

        if let error {
            if characteristic.uuid == jointTargetsUUID {
                jointWriteInFlight = false
            }
            if characteristic.uuid == cartesianTargetsUUID {
                cartesianWriteInFlight = false
            }

            // If we were trying to disconnect, continue anyway.
            if lastControlCommand == .releaseAndDisconnect {
                if let connectedPeripheral {
                    central?.cancelPeripheralConnection(connectedPeripheral)
                }
                disconnectAfterNextWrite = false
                lastControlCommand = .none
                return
            }

            lastError = error.localizedDescription
            return
        }

        switch characteristic.uuid {
        case controlPointUUID:
            let completedCommand = lastControlCommand
            lastControlCommand = .none

            switch completedCommand {
            case .take:
                readOwnerStatus()
                writeControlModeNow(controlMode)
            case .release:
                hasControl = false
                readOwnerStatus()
            case .none:
                break
            case .releaseAndDisconnect:
                if let connectedPeripheral {
                    central?.cancelPeripheralConnection(connectedPeripheral)
                }
                disconnectAfterNextWrite = false
            case .setMode:
                flushActiveTargetsNow()
            }

        case jointTargetsUUID:
            jointWriteInFlight = false
            flushPendingTargetsIfNeeded()

        case cartesianTargetsUUID:
            cartesianWriteInFlight = false
            flushPendingTargetsIfNeeded()

        default:
            break
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        _ = peripheral

        if let error {
            lastError = error.localizedDescription
            return
        }

        guard characteristic.uuid == ownerTokenUUID,
              let data = characteristic.value,
              data.count >= 1 else {
            return
        }

        hasControl = (data[0] != 0)
        if hasControl {
            flushPendingTargetsIfNeeded()
        }
    }

    private func bluetoothStateMessage(for state: CBManagerState) -> String {
        switch state {
        case .unknown:
            return "Bluetooth state unknown"
        case .resetting:
            return "Bluetooth is resetting"
        case .unsupported:
            return "Bluetooth unsupported on this Mac"
        case .unauthorized:
            return "Bluetooth unauthorized (check Privacy & Security → Bluetooth)"
        case .poweredOff:
            return "Bluetooth is powered off"
        case .poweredOn:
            return "Bluetooth ready"
        @unknown default:
            return "Bluetooth state unavailable"
        }
    }

    func peripheralIsReady(toSendWriteWithoutResponse peripheral: CBPeripheral) {
        guard peripheral.identifier == connectedPeripheral?.identifier else { return }
        flushPendingTargetsIfNeeded()
    }
}
