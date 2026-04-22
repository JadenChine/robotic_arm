import Combine
import CoreBluetooth
import Foundation

final class BLEManager: NSObject, ObservableObject {
    @Published var discoveredPeripherals: [CBPeripheral] = []
    @Published var connectionState: String = "Bluetooth not ready"
    @Published var activePeripheralName: String = "Not connected"
    @Published var hasControl: Bool = false
    @Published var currentOwnerToken: UInt32 = 0
    @Published var myOwnerToken: UInt32 = 0
    @Published var lastError: String = ""

    private var central: CBCentralManager?
    private var connectedPeripheral: CBPeripheral?

    private var controlPointCharacteristic: CBCharacteristic?
    private var ownerTokenCharacteristic: CBCharacteristic?
    private var jointTargetsCharacteristic: CBCharacteristic?

    private var pendingJointTargetsDegrees: [Double] = Array(repeating: 0.0, count: 6)
    private var pendingJointTargetsDirty = false
    private var awaitingTakeControlConfirmation = false

    private var jointSendTimer: Timer?
    private var ownerPollTimer: Timer?

    private enum LastControlCommand {
        case none
        case take
        case release
    }

    private var lastControlCommand: LastControlCommand = .none

    private let armServiceUUID = CBUUID(string: "FFF0")
    private let controlPointUUID = CBUUID(string: "FFF1")
    private let ownerTokenUUID = CBUUID(string: "FFF2")
    private let jointTargetsUUID = CBUUID(string: "FFF3")

    override init() {
        super.init()
        central = CBCentralManager(delegate: self, queue: .main)

        jointSendTimer = Timer.scheduledTimer(withTimeInterval: 0.05, repeats: true) {
            [weak self] _ in
            self?.flushPendingJointTargetsIfNeeded()
        }

        ownerPollTimer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) {
            [weak self] _ in
            self?.pollOwnerTokenIfPossible()
        }
    }

    deinit {
        jointSendTimer?.invalidate()
        ownerPollTimer?.invalidate()
    }

    func startScan() {
        guard let central else { return }
        guard central.state == .poweredOn else {
            connectionState = "Bluetooth is not powered on"
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
        central?.cancelPeripheralConnection(peripheral)
    }

    func takeControl() {
        guard let peripheral = connectedPeripheral,
              let characteristic = controlPointCharacteristic else {
            lastError = "Not connected to control characteristic"
            return
        }

        let payload = Data([0x01])
        lastControlCommand = .take
        awaitingTakeControlConfirmation = true
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

    func updateJointTargetsDegrees(_ jointAngles: [Double]) {
        guard jointAngles.count == 6 else {
            lastError = "Joint array must contain exactly 6 values"
            return
        }

        pendingJointTargetsDegrees = jointAngles
        pendingJointTargetsDirty = true
    }

    private func flushPendingJointTargetsIfNeeded() {
        guard pendingJointTargetsDirty else { return }
        guard hasControl else { return }
        guard let peripheral = connectedPeripheral,
              let characteristic = jointTargetsCharacteristic else {
            return
        }

        let packet = encodeJointTargetsPacket(pendingJointTargetsDegrees)
        let writeType: CBCharacteristicWriteType = characteristic.properties.contains(.writeWithoutResponse) ? .withoutResponse : .withResponse

        peripheral.writeValue(packet, for: characteristic, type: writeType)
        pendingJointTargetsDirty = false
    }

    private func encodeJointTargetsPacket(_ jointAngles: [Double]) -> Data {
        var data = Data(capacity: 12)

        for angle in jointAngles {
            let deciDegrees = Int16(max(-900, min(900, Int(round(angle * 10.0)))))
            var littleEndianValue = deciDegrees.littleEndian
            withUnsafeBytes(of: &littleEndianValue) { bytes in
                data.append(contentsOf: bytes)
            }
        }

        return data
    }

    private func pollOwnerTokenIfPossible() {
        guard connectedPeripheral != nil else { return }
        readOwnerToken()
    }

    private func readOwnerToken() {
        guard let peripheral = connectedPeripheral,
              let characteristic = ownerTokenCharacteristic else {
            return
        }

        peripheral.readValue(for: characteristic)
    }

    private func resetConnectionState() {
        connectedPeripheral = nil
        controlPointCharacteristic = nil
        ownerTokenCharacteristic = nil
        jointTargetsCharacteristic = nil
        activePeripheralName = "Not connected"
        currentOwnerToken = 0
        myOwnerToken = 0
        hasControl = false
        awaitingTakeControlConfirmation = false
        lastControlCommand = .none
        pendingJointTargetsDirty = false
    }
}

extension BLEManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            connectionState = "Bluetooth ready"
            startScan()
        case .poweredOff:
            connectionState = "Bluetooth is powered off"
            resetConnectionState()
        case .resetting:
            connectionState = "Bluetooth resetting"
            resetConnectionState()
        case .unauthorized:
            connectionState = "Bluetooth unauthorized"
            resetConnectionState()
        case .unsupported:
            connectionState = "BLE unsupported on this Mac"
            resetConnectionState()
        case .unknown:
            fallthrough
        @unknown default:
            connectionState = "Bluetooth state unknown"
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
            peripheral.discoverCharacteristics([controlPointUUID, ownerTokenUUID, jointTargetsUUID], for: service)
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
                ownerTokenCharacteristic = characteristic
            case jointTargetsUUID:
                jointTargetsCharacteristic = characteristic
            default:
                break
            }
        }

        if controlPointCharacteristic != nil,
           ownerTokenCharacteristic != nil,
           jointTargetsCharacteristic != nil {
            connectionState = "Connected and ready"
            readOwnerToken()
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic,
                    error: Error?) {
        _ = peripheral

        if let error {
            lastError = error.localizedDescription
            if lastControlCommand == .take {
                awaitingTakeControlConfirmation = false
            }
            return
        }

        switch characteristic.uuid {
        case controlPointUUID:
            switch lastControlCommand {
            case .take:
                awaitingTakeControlConfirmation = true
                readOwnerToken()
            case .release:
                myOwnerToken = 0
                hasControl = false
                readOwnerToken()
            case .none:
                break
            }
            lastControlCommand = .none

        case jointTargetsUUID:
            break

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
              data.count >= 4 else {
            return
        }

        let token = data.withUnsafeBytes { rawBuffer in
            rawBuffer.load(as: UInt32.self)
        }.littleEndian

        currentOwnerToken = token

        if awaitingTakeControlConfirmation {
            myOwnerToken = token
            awaitingTakeControlConfirmation = false
        }

        hasControl = (myOwnerToken != 0 && currentOwnerToken == myOwnerToken)
    }
}
