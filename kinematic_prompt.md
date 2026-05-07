# Task: Add Cartesian (XYZ) Control Mode (BLE + Firmware + macOS App)

## Context (what exists today)
This repo already contains:
- Pico/Pico W firmware that controls a 6-DOF arm via joint target setpoints and a 20 Hz control loop: [src/arm/main.c](src/arm/main.c)
- BLE GATT server running on core1 (Pico W), with ownership semantics and an event queue to core0:  
  - Interface + event types: [src/arm/src/ble/arm_ble_interface.h](src/arm/src/ble/arm_ble_interface.h)  
  - Implementation: [src/arm/src/ble/arm_ble_interface.c](src/arm/src/ble/arm_ble_interface.c)  
  - GATT definition: [src/arm/src/ble/arm_control_service.gatt](src/arm/src/ble/arm_control_service.gatt)
- macOS SwiftUI controller app that:
  - scans for service `FFF0`, connects, and discovers `FFF1/FFF2/FFF3`
  - “Take Control” (`0x01`) / “Release” (`0x02`) via `FFF1`
  - streams joint targets to `FFF3` as 6×`int16` little-endian **deci-degrees** (0.1° units)
  - current UI is 6 joint sliders: [src/mac-app/src/ContentView.swift](src/mac-app/src/ContentView.swift)
  - BLE logic is in: [src/mac-app/src/BLEManager.swift](src/mac-app/src/BLEManager.swift)
- A Python simulation model describing the desired coordinate system and motion conventions (4-DOF, 3 links). Use it as the **reference spec** for frames/limits/signs/reach, not as code to copy: [sim/README.md](sim/README.md)

## Goal
Add a second control mode: **Cartesian (XYZ) tip control**, so I can move the arm tip/end-effector position using X/Y/Z sliders instead of directly commanding the main arm joints.

This requires coordinated changes in:
1) macOS SwiftUI app UI + BLE protocol usage
2) Pico W firmware BLE service + application control loop
3) a new kinematics implementation (IK/FK/Jacobian) consistent with the simulation’s coordinate system

## Key constraints / modeling assumptions
### Degrees of freedom used for Cartesian IK
The simulation is 4-DOF (base yaw + 3 pitch joints) and matches the real arm’s “positioning” behavior.

For Cartesian control, **ignore wrist/gripper joints** in the IK math:
- Treat wrist/gripper as not affecting XYZ positioning (treat the last link as a single rigid link as in the sim).
- On the real arm, the joints used by IK correspond to these physical joints:
  - Sim Joint 1 (Base/Shoulder Yaw) → real Joint 6
  - Sim Joint 2 (Shoulder Pitch) → real Joint 5
  - Sim Joint 3 (Elbow 1) → real Joint 4
  - Sim Joint 4 (Elbow 2) → real Joint 3
- Real Joints 1–2 are wrist/gripper; keep them **manually controllable** even while in Cartesian mode.

### Cartesian mode UI controls
In Cartesian mode, the macOS app should present:
- 3 sliders: X, Y, Z (tip target position)
- 2 additional sliders to manually control wrist/gripper joints (real Joint 1 and Joint 2)

Total: **5 sliders** visible in Cartesian mode.

Then there must be a button to revert back to individual joint control mode and bring back the original sliders.

### Coordinate system and units
Use the coordinate system described in the simulation documentation:
- Origin and axes per [sim/README.md](sim/README.md)
- Link lengths and base offset per sim doc (unless you decide a calibration approach is needed)

XYZ units must be mm and must be consistent end-to-end (UI, BLE packet, firmware).

## UX / UI requirements (macOS)
Add a button labeled **“Cartesian Control”** that switches the UI into Cartesian mode.
- Keep the existing look/structure: this should be a simple mode switch, not a full redesign.
- In Cartesian mode:
  - show X/Y/Z sliders (with clear units in labels)
  - show two sliders for wrist/gripper (Joint 1 and Joint 2) still in degrees like today
- In Joint mode:
  - keep the current 6-joint slider behavior intact

## BLE protocol requirements
Today the BLE contract is:
- service `FFF0`
- `FFF1` control point: TAKE (`0x01`), RELEASE (`0x02`)
- `FFF2` owner status read: `0x01` if this connection owns control
- `FFF3` joint targets write: 12 bytes (6×int16 deci-deg)

You must add a BLE mechanism to:
1) switch firmware between joint-target mode and Cartesian-target mode, and
2) send Cartesian targets (XYZ) when in Cartesian mode.

You may implement this by:
- extending `FFF1` control point with new commands (recommended), and
- adding a new characteristic for Cartesian targets (recommended), OR by another clean, minimal protocol extension.

Ownership rules must remain consistent:
- only the owner affects motion
- safe pose behavior on owner release/disconnect remains (or propose a safer Cartesian-equivalent)

## Firmware behavior requirements
### Control loop integration
Firmware currently runs a 20 Hz “dirty setpoint” loop on core0 in [src/arm/main.c](src/arm/main.c).
Add a clean mode separation:
- **Joint mode**: behave exactly as today (apply `FFF3` joint targets directly).
- **Cartesian mode**:
  - accept XYZ target updates from BLE
  - compute the required angles for joints 6/5/4/3 (the IK joints)
  - apply those targets into the existing joint target registers so the same clamp/transform/deadband pipeline is used
  - accept manual targets for joints 1–2 (wrist/gripper) concurrently just like it is currently

### Kinematics implementation location
Add a new module under something like:
- `src/arm/src/kinematics/` (or another sensible location)

The kinematics module should include at minimum:
- forward kinematics for the 4-DOF chain (to compute tip XYZ from angles)
- inverse kinematics strategy (you choose):
  - Jacobian-based iterative IK (e.g., damped least squares) is acceptable
  - closed-form IK is acceptable if you can keep it robust and consistent with constraints
- joint limits consistent with firmware/app limits and the sim doc
- avoid jitter/sudden jumps when the XYZ target changes slightly
- define how you handle unreachable targets (clamp to nearest reachable, or reject and hold last valid, etc.)

### Mapping and conventions (important)
The IK must output angles in the same “logical joint space” that the existing app/firmware uses (degrees → deci-deg), so that existing per-joint clamp/direction handling remains correct and consistent.

## Non-goals (to keep scope contained)
- Do not add orientation control (roll/pitch/yaw of the tool) unless explicitly requested later.
- Do not redesign BLE ownership model.
- Do not introduce heavy UI complexity beyond a mode switch + sliders.

## Acceptance criteria
- Joint mode continues to work exactly as before.
- Cartesian mode:
  - switching modes works without rebooting the Pico W
  - X/Y/Z slider changes cause the arm tip to move in the expected direction per sim coordinate conventions
  - wrist/gripper sliders still work (and do not break the IK joints)
- BLE remains stable under continuous slider drag (similar to current “send latest at ~60 Hz” behavior).
- Code is organized (kinematics not stuffed into main loop), and builds cleanly.

## Implementation notes (guidance, not strict)
- Keep changes somewhat consistent with the existing architecture described in:
  - firmware reference: [src/arm/README.md](src/arm/README.md)
  - mac app reference: [src/mac-app/README.md](src/mac-app/README.md)
- The sim README is the source of truth for coordinate system conventions: [sim/README.md](sim/README.md)

## Clarifying Questions
If you encounter any ambiguities or concerns, please ask me clarifying questions. Your job is to implement my goals in these features in the most efficient and smartest way possible. If you find that any suggestions in this prompt are unreasonable, suboptimal, or unclear, ask questions so I can provide clarification. Feel free to suggest new strategies or better implementation goals.