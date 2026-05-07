import SwiftUI

struct ContentView: View {
    @EnvironmentObject var bleManager: BLEManager
    @State private var jointAngles: [Double] = Array(repeating: 0.0, count: 6)

    private let jointRangesDegrees: [(min: Double, max: Double)] = [
        (min: -120.0, max: 10.0),
        (min: -120.0, max: 120.0),
        (min: -120.0, max: 120.0),
        (min: -120.0, max: 120.0),
        (min: -90.0, max: 90.0),
        (min: -120.0, max: 120.0),
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

                Button("Center Pose") {
                    jointAngles = Array(repeating: 0.0, count: 6)
                    bleManager.updateJointTargetsDegrees(jointAngles)
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

            GroupBox("Joint Targets (degrees)") {
                VStack(spacing: 12) {
                    ForEach(0..<6, id: \.self) { index in
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
                                in: range.min...range.max,
                                step: 0.1
                            )
                        }
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
}

