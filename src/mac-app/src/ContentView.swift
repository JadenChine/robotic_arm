import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @State private var jointAngles: [Double] = Array(repeating: 0.0, count: 6)
    @State private var cartesianTargetsMM: [Double] = [0.0, 0.0, 429.0]

    private let jointRangesDegrees: [(min: Double, max: Double)] = [
        (min: -120.0, max: 10.0),
        (min: -120.0, max: 120.0),
        (min: -120.0, max: 120.0),
        (min: -120.0, max: 120.0),
        (min: -90.0, max: 90.0),
        (min: -120.0, max: 120.0),
    ]

    private let cartesianRangesMM: [(label: String, min: Double, max: Double)] = [
        (label: "X", min: -364.0, max: 364.0),
        (label: "Y", min: -364.0, max: 364.0),
        (label: "Z", min: 0.0, max: 429.0),
    ]

    var body: some View {
        VStack(alignment: .leading, spacing: 16) {
            Text("Robotic Arm BLE Control")
                .font(.system(size: 30, weight: .bold, design: .rounded))

            GroupBox {
                VStack(alignment: .leading, spacing: 8) {
                    statusRow(label: "State", value: bleManager.connectionState)
                    statusRow(label: "Peripheral", value: bleManager.activePeripheralName)
                    statusRow(label: "Control", value: bleManager.hasControl ? "OWNER" : "NOT OWNER")
                    statusRow(label: "Mode", value: bleManager.controlMode.displayName)

                    if !bleManager.lastError.isEmpty {
                        Text("Last BLE error: \(bleManager.lastError)")
                            .font(.caption)
                            .foregroundStyle(.red)
                    }
                }
            }

            HStack(spacing: 10) {
                Button("Scan") {
                    bleManager.startScan()
                }

                Button("Stop Scan") {
                    bleManager.stopScan()
                }

                Button("Disconnect") {
                    bleManager.disconnect()
                }
                .disabled(!bleManager.isConnected)

                Button("Take Control") {
                    bleManager.takeControl()
                }
                .buttonStyle(.borderedProminent)

                Button("Release") {
                    bleManager.releaseControl()
                }

                if bleManager.controlMode == .joint {
                    Button("Cartesian Control") {
                        enterCartesianMode()
                    }
                } else {
                    Button("Joint Control") {
                        enterJointMode()
                    }
                }

                Button("Center Pose") {
                    centerPose()
                }
            }

            GroupBox("Discovered Arm Peripherals") {
                if bleManager.discoveredPeripherals.isEmpty {
                    Text("No matching peripherals found yet")
                        .foregroundStyle(.secondary)
                } else {
                    List(bleManager.discoveredPeripherals, id: \.identifier) { peripheral in
                        HStack {
                            VStack(alignment: .leading) {
                                Text(peripheral.name ?? "Unnamed")
                                Text(peripheral.identifier.uuidString)
                                    .font(.caption)
                                    .foregroundStyle(.secondary)
                            }
                            Spacer()
                            Button("Connect") {
                                bleManager.connect(to: peripheral)
                            }
                        }
                    }
                    .frame(height: 160)
                }
            }

            if bleManager.controlMode == .joint {
                GroupBox("Joint Targets (degrees)") {
                    VStack(spacing: 12) {
                        ForEach(0..<6, id: \.self) { index in
                            jointSlider(index: index)
                        }
                    }
                }
            } else {
                GroupBox("Cartesian Tip Target (mm)") {
                    VStack(spacing: 12) {
                        ForEach(0..<3, id: \.self) { index in
                            cartesianSlider(index: index)
                        }
                    }
                }

                GroupBox("Wrist / Gripper Targets (degrees)") {
                    VStack(spacing: 12) {
                        jointSlider(index: 0)
                        jointSlider(index: 1)
                    }
                }
            }

            if !bleManager.hasControl {
                Text("Sliders always transmit; the arm applies targets only from the current owner. Press Take Control to become the owner.")
                    .font(.callout)
                    .foregroundStyle(.secondary)
            }
        }
        .padding(18)
        .onAppear {
            bleManager.startScan()
            bleManager.updateJointTargetsDegrees(jointAngles)
            bleManager.updateCartesianTargetsMM(cartesianTargetsMM)
        }
    }

    @ViewBuilder
    private func statusRow(label: String, value: String) -> some View {
        HStack {
            Text(label + ":")
                .foregroundStyle(.secondary)
            Spacer()
            Text(value)
                .multilineTextAlignment(.trailing)
                .textSelection(.enabled)
        }
    }

    @ViewBuilder
    private func jointSlider(index: Int) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Text("Joint \(index + 1)")
                Spacer()
                Text(String(format: "%.1f°", jointAngles[index]))
                    .monospacedDigit()
            }

            let range = jointRangesDegrees[index]

            Slider(
                value: Binding(
                    get: { jointAngles[index] },
                    set: { newValue in
                        var updatedAngles = jointAngles
                        updatedAngles[index] = newValue
                        jointAngles = updatedAngles
                        bleManager.updateJointTargetsDegrees(updatedAngles)
                    }
                ),
                // A continuous slider avoids thousands of automatic macOS ticks.
                // The value label and BLE packet still round to 0.1 degrees.
                in: range.min...range.max
            )
        }
    }

    @ViewBuilder
    private func cartesianSlider(index: Int) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            HStack {
                Text(cartesianRangesMM[index].label)
                Spacer()
                Text(String(format: "%.0f mm", cartesianTargetsMM[index]))
                    .monospacedDigit()
            }

            let range = cartesianRangesMM[index]

            Slider(
                value: Binding(
                    get: { cartesianTargetsMM[index] },
                    set: { newValue in
                        updateCartesianTarget(axisIndex: index, value: newValue)
                    }
                ),
                in: range.min...range.max
            )
        }
    }

    private func enterCartesianMode() {
        cartesianTargetsMM = ArmKinematics.tipPositionMM(fromLogicalJointAngles: jointAngles)
        bleManager.updateCartesianTargetsMM(cartesianTargetsMM)
        bleManager.setControlMode(.cartesian)
    }

    private func enterJointMode() {
        bleManager.setControlMode(.joint)
        bleManager.updateJointTargetsDegrees(jointAngles)
    }

    private func centerPose() {
        if bleManager.controlMode == .joint {
            jointAngles = Array(repeating: 0.0, count: 6)
            cartesianTargetsMM = ArmKinematics.tipPositionMM(fromLogicalJointAngles: jointAngles)
            bleManager.updateJointTargetsDegrees(jointAngles)
        } else {
            cartesianTargetsMM = [0.0, 0.0, 429.0]
            var updatedAngles = ArmKinematics.logicalJointAngles(forCartesianTargetMM: cartesianTargetsMM, seedJointAngles: jointAngles)
            updatedAngles[0] = 0.0
            updatedAngles[1] = 0.0
            jointAngles = updatedAngles
            bleManager.updateCartesianTargetsMM(cartesianTargetsMM)
            bleManager.updateJointTargetsDegrees(jointAngles)
        }
    }

    private func updateCartesianTarget(axisIndex: Int, value: Double) {
        var updatedTargets = cartesianTargetsMM
        updatedTargets[axisIndex] = value
        cartesianTargetsMM = updatedTargets
        jointAngles = ArmKinematics.logicalJointAngles(forCartesianTargetMM: updatedTargets, seedJointAngles: jointAngles)
        bleManager.updateCartesianTargetsMM(updatedTargets)
    }
}
