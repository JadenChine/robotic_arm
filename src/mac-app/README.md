# macOS BLE Arm Controller (SwiftUI)

This folder contains a native SwiftUI macOS app that connects to the robotic arm (Pico W firmware) over Bluetooth Low Energy (BLE) and streams joint target angles or Cartesian XYZ tip targets.

It is designed for quick interactive control while still being safe on BLE bandwidth and robust under load.

## What It Accomplishes

- Finds compatible peripherals by scanning for a custom BLE service (`0xFFF0`).
- Connects and discovers the arm’s control characteristics.
- Lets you take/release “control ownership” via a control-point write.
- Streams a 6-joint target pose from the UI sliders.
- Switches between six-joint control and Cartesian XYZ tip control.
- Streams X/Y/Z targets in millimeters while still allowing manual wrist/gripper control.
- Keeps UI and BLE ownership state in sync by polling the owner-status characteristic.

## High-Level Architecture

There are three main pieces:

1. `ContentView.swift`
  - SwiftUI UI: status panel, scan/connect buttons, device list, mode switch, and mode-specific sliders.
  - Maintains local joint and Cartesian slider state and forwards updates to the BLE layer.

2. `BLEManager.swift`
   - Owns the CoreBluetooth `CBCentralManager` and the active `CBPeripheral`.
   - Discovers characteristics, performs reads/writes, tracks connection/ownership state.
   - Implements a small “send queue” so slider updates transmit at a controlled rate.

3. `ArmKinematics.swift`
  - Lightweight FK/IK helper used by the UI to initialize XYZ from the current joint pose and keep estimated joint sliders coherent when returning from Cartesian mode.
  - Firmware remains authoritative for physical motion.

## BLE Protocol Used

The app expects the arm peripheral to expose service `0xFFF0` with these characteristics:

- `0xFFF1` — Control Point (write)
  - `0x01` = TAKE_CONTROL
  - `0x02` = RELEASE_CONTROL
  - `0x03, mode` = SET_CONTROL_MODE
    - `0x00` = joint target mode
    - `0x01` = Cartesian XYZ mode

- `0xFFF2` — Owner Status (read)
  - `0x01` means *this* connection is the current owner
  - `0x00` means not the owner

- `0xFFF3` — Joint Targets (write)
  - Payload is 12 bytes total: 6 × `int16` little-endian
  - Units are deci-degrees (0.1°): value = `round(deg * 10)`
  - Range enforced by the app is per-joint (deci-degrees):
    - Joint 1: `[-1200, +100]` ([-120.0°, +10.0°])
    - Joint 2: `[-1200, +1200]` ([-120.0°, +120.0°])
    - Joint 3: `[-1200, +1200]` ([-120.0°, +120.0°])
    - Joint 4: `[-1200, +1200]` ([-120.0°, +120.0°])
    - Joint 5: `[-900, +900]` ([-90.0°, +90.0°])
    - Joint 6: `[-1200, +1200]` ([-120.0°, +120.0°])

- `0xFFF4` — Cartesian Targets (write)
  - Payload is 6 bytes total: 3 × `int16` little-endian
  - Units are millimeters
  - Order is X, Y, Z in the simulation coordinate frame
  - App slider ranges:
    - X: `[-364, +364] mm`
    - Y: `[-364, +364] mm`
    - Z: `[0, 429] mm`

In Cartesian mode, `0xFFF3` still sends all six joint values for packet compatibility, but firmware applies only real Joints 1 and 2. Firmware IK owns real Joints 3-6.

## How Slider Updates Are Sent (Mid-Drag)

The UI sliders update continuously while you drag.

- In joint mode, every slider movement updates `jointAngles` and calls `BLEManager.updateJointTargetsDegrees(...)`.
- In Cartesian mode, X/Y/Z slider movement calls `BLEManager.updateCartesianTargetsMM(...)`, while Joint 1 and Joint 2 sliders call `BLEManager.updateJointTargetsDegrees(...)`.
- `BLEManager` stores the latest active target packet as “pending” and marks it dirty.
- A timer flushes pending targets every ~0.0167 seconds (60 Hz). If there were multiple slider changes within that window, only the latest pose is sent.
- The send loop honors CoreBluetooth backpressure (`canSendWriteWithoutResponse`) so target writes do not build up in a local queue.
- When the joint-target characteristic supports it, the app uses `writeWithoutResponse` for lower latency; otherwise it falls back to `withResponse`.

This design gives you responsive mid-slide control while keeping write pressure bounded to what the BLE stack can actually transmit.

## Control Ownership Model

The firmware enforces a single “owner” at a time:

- The app only transmits mode and target packets when this connection is the owner.
- Press **Take Control** to attempt to become the owner.
- Press **Release** to relinquish ownership.

Implementation details:

- Owner status is read from `0xFFF2`.
- The app polls owner status every 0.5 seconds so the UI stays accurate even if another central takes control.
- After a successful TAKE_CONTROL write, the app writes the selected control mode and immediately sends the current target packet for that mode.

## App States

The UI shows a human-readable `State` value driven by CoreBluetooth events.

### Bluetooth availability

- **Bluetooth ready**: macOS Bluetooth is powered on.
- **Bluetooth powered off**: Bluetooth is disabled.
- **Bluetooth unauthorized**: permission is denied (see Troubleshooting).
- **Bluetooth unsupported / resetting / unknown**: uncommon system-level states.

### Connection lifecycle

- **Scanning for arm peripheral...**: actively scanning for peripherals advertising `0xFFF0`.
- **Connecting to <name>...**: a Connect button was pressed.
- **Connected, discovering services...**: connected at the link layer; discovering GATT.
- **Connected and ready**: all four characteristics (`FFF1`, `FFF2`, `FFF3`, `FFF4`) are discovered.
- **Disconnected**: link lost or user disconnected; the app auto-returns to scanning.

### Control state

- **OWNER**: `0xFFF2` reads `0x01`.
- **NOT OWNER**: `0xFFF2` reads `0x00`.

## UI Overview

- **Status box**
  - State, active peripheral name, ownership indicator, selected mode, and last BLE error.

- **Action buttons**
  - Scan / Stop Scan
  - Disconnect (disabled unless connected)
  - Take Control / Release
  - Cartesian Control / Joint Control mode switch
  - Center Pose (mode-aware pose reset)

- **Discovered Arm Peripherals**
  - List of peripherals found via scan (deduplicated by identifier).
  - Connect button per row.

- **Joint Targets** (joint mode)
  - Six sliders with 0.1° step and per-joint ranges:
    - Joint 1: -120.0° to +10.0°
    - Joint 2–4: -120.0° to +120.0°
    - Joint 5: -90.0° to +90.0°
    - Joint 6: -120.0° to +120.0°
  - The value label updates live while dragging.

- **Cartesian Tip Target** (Cartesian mode)
  - X, Y, and Z sliders in millimeters.
  - X/Y range: -364 mm to +364 mm.
  - Z range: 0 mm to 429 mm.

- **Wrist / Gripper Targets** (Cartesian mode)
  - Real Joint 1 and Joint 2 sliders remain available in degrees.
  - These sliders do not affect the IK-controlled positioning joints.

## Run

From this directory:

```bash
swift run
```

If macOS asks for Bluetooth permissions, allow access.

## Troubleshooting

### “Bluetooth unauthorized”

- Ensure Bluetooth is enabled in macOS.
- Go to **System Settings → Privacy & Security → Bluetooth** and enable access for the app.
- If you previously clicked “Don’t Allow”, reset the permission and re-run to re-trigger the prompt:
  - `tccutil reset BluetoothAlways`

Note: When running via `swift run` from VS Code/Terminal, the Bluetooth permission may be attributed to the launching app (for example **Visual Studio Code** or **Terminal**) depending on macOS version.

### Sliders move but the arm doesn’t

- Check **Control** reads **OWNER**.
- If it’s **NOT OWNER**, press **Take Control**.
- If another device/app is connected, it may currently own control.

### Cartesian sliders move but XYZ does not affect the arm

- Check **Mode** reads **Cartesian**.
- Confirm firmware exposes `0xFFF4`; older firmware will not reach “Connected and ready”.
- If the requested XYZ point is outside the reachable workspace, firmware holds the last valid IK pose instead of jumping.

### The arm jumps when taking control

This is expected: on successful ownership, the app immediately pushes the current mode and slider pose so the robot matches what you see in the UI.
