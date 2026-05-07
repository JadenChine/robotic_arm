# Session Handoff: Cartesian XYZ Control Mode

## Purpose of This Document

This file is a full handoff summary for continuing this work in a new session on a different machine.

It captures:

- what the user asked for,
- the architectural decisions made during this session,
- exactly what was implemented,
- what could and could not be verified on this machine,
- and the concrete next steps for finishing validation and any follow-up fixes.

The goal is that a new session should be able to pick up from here without needing to rediscover the reasoning or inspect the entire repo from scratch.

---

## Original User Request

The user asked to add a second control mode for the robotic arm: Cartesian XYZ tip control, while preserving the existing joint-angle control mode.

### Existing system before this session

- Firmware in `src/arm` already supported BLE ownership, six-joint target streaming, and a 20 Hz control loop.
- The Pico W BLE service already exposed:
	- `FFF0` service
	- `FFF1` control point with TAKE (`0x01`) and RELEASE (`0x02`)
	- `FFF2` owner status read
	- `FFF3` joint targets as 6 x `int16` deci-degrees
- The macOS app already scanned, connected, took/released ownership, and streamed six joint sliders.
- The Python simulation in `sim/` defined the reference coordinate frame, link lengths, and motion conventions for a 4-DOF positioning chain.

### Functional goal requested by the user

Add Cartesian mode so the arm tip can be moved using X/Y/Z sliders instead of only per-joint sliders.

### Modeling constraints from the user

The user explicitly specified that Cartesian IK should use the simulation's 4-DOF positioning chain and ignore wrist/gripper joints in the IK math.

The mapping given by the user was:

- Sim Joint 1 (base yaw) -> real Joint 6
- Sim Joint 2 (shoulder pitch) -> real Joint 5
- Sim Joint 3 (elbow 1) -> real Joint 4
- Sim Joint 4 (elbow 2) -> real Joint 3

Real Joints 1 and 2 must remain manually controllable even in Cartesian mode.

### UI requirements from the user

- Add a button labeled `Cartesian Control` to switch modes.
- Joint mode must keep the original six-joint slider UI.
- Cartesian mode must show:
	- X slider
	- Y slider
	- Z slider
	- real Joint 1 slider
	- real Joint 2 slider
- There must also be a way to switch back to joint mode.

### BLE requirements from the user

The user allowed extending the existing BLE protocol and specifically suggested:

- extending `FFF1` with mode-switch commands, and
- adding a new characteristic for Cartesian targets.

Ownership rules had to remain unchanged:

- only the current owner affects motion,
- non-owner writes remain ignored,
- safe pose on release/disconnect remains in place.

### Additional documentation request from the user

The user asked for an in-depth README in the new firmware kinematics directory describing the math and implementation.

---

## Clarifications and Decisions Made During This Session

The user answered three clarification points during this session.

### Confirmed protocol design

The user approved the recommended BLE extension:

- `FFF1` command `0x03` with payload `[0x03, mode]`
- `mode = 0` for joint mode
- `mode = 1` for Cartesian mode
- new `FFF4` characteristic for Cartesian targets
- `FFF4` payload = `3 x int16` little-endian millimeters in the order `X, Y, Z`

### Confirmed slider ranges

The user approved the simulation-derived workspace ranges for the macOS app:

- X: `-364 ... 364 mm`
- Y: `-364 ... 364 mm`
- Z: `0 ... 429 mm`

### Confirmed unreachable-target behavior

The user approved the recommended behavior:

- clamp obvious workspace bounds first,
- seed IK from the last valid solution,
- if solving fails, hold the last valid IK pose,
- keep wrist/gripper manual updates active even while IK is holding.

---

## High-Level Implementation Strategy Chosen

The implementation was intentionally conservative and architecture-aligned with the existing repo.

### Core principles used

1. Preserve joint mode behavior as much as possible.
2. Keep kinematics logic out of BLE handlers and out of the HAL.
3. Reuse the existing firmware clamp/transform/deadband/UART output path.
4. Extend the BLE protocol minimally instead of replacing it.
5. Keep real wrist/gripper control in the existing joint-target path for compatibility.

### Resulting design

#### Firmware

- Joint mode still applies all six values from `FFF3` exactly as before.
- Cartesian mode uses:
	- `FFF4` for XYZ targets,
	- `FFF3` only for real Joints 1 and 2,
	- a new kinematics module to compute sim-space IK for the positioning joints,
	- mapping from sim joints into real Joints 6/5/4/3,
	- the existing `g_joint_target_deci_deg[]` registers and existing servo output path.

#### macOS app

- A simple mode switch was added instead of redesigning the UI.
- BLEManager now knows how to:
	- write control mode via `FFF1`,
	- discover `FFF4`,
	- send Cartesian targets at the same coalesced rate as joint targets.
- A lightweight Swift kinematics helper was added so the UI can:
	- initialize XYZ from the current joint pose when entering Cartesian mode,
	- keep estimated joint values coherent when returning to joint mode.

---

## Files Changed During This Session

### Firmware files changed

- `src/arm/CMakeLists.txt`
- `src/arm/README.md`
- `src/arm/docs/BLE_README.md`
- `src/arm/main.c`
- `src/arm/src/ble/arm_ble_interface.h`
- `src/arm/src/ble/arm_ble_interface.c`
- `src/arm/src/ble/arm_control_service.gatt`

### New firmware files added

- `src/arm/src/kinematics/arm_kinematics.h`
- `src/arm/src/kinematics/arm_kinematics.c`
- `src/arm/src/kinematics/README.md`

### macOS files changed

- `src/mac-app/README.md`
- `src/mac-app/src/BLEManager.swift`
- `src/mac-app/src/ContentView.swift`

### New macOS file added

- `src/mac-app/src/ArmKinematics.swift`

### Pre-existing unrelated worktree changes that were intentionally not touched

- `kinematic_prompt.md` is deleted in the worktree.
- `.python-version` is present as a new file.

Those were left alone because they are unrelated to the requested feature.

---

## Detailed Summary of What Was Implemented

## 1. BLE Protocol Extension

### `FFF1` control point

The control point now supports a third command:

- `0x01` = TAKE_CONTROL
- `0x02` = RELEASE_CONTROL
- `0x03, mode` = SET_CONTROL_MODE

Mode values:

- `0x00` = joint mode
- `0x01` = Cartesian mode

Mode writes are owner-only.

### `FFF4` Cartesian target characteristic

A new characteristic was added:

- UUID: `FFF4`
- Properties: `WRITE | WRITE_WITHOUT_RESPONSE | DYNAMIC`
- Payload: `3 x int16` little-endian millimeters
- Order: `X`, `Y`, `Z`

### Ownership behavior retained

Target stream writes remain ownership-gated exactly like the old protocol:

- non-owner target writes return success but are ignored,
- owner disconnect still triggers safe pose,
- owner release still triggers safe pose.

### Event model extension

The BLE event struct now supports:

- set joint targets,
- safe pose,
- set control mode,
- set Cartesian target.

This is implemented in:

- `src/arm/src/ble/arm_ble_interface.h`
- `src/arm/src/ble/arm_ble_interface.c`

---

## 2. Firmware Kinematics Module

A new firmware-side module was added in `src/arm/src/kinematics/`.

### What it does

- Forward kinematics for the 4-DOF simulation-compatible positioning chain.
- Inverse kinematics for XYZ targets.
- Workspace clamping.
- Last-valid-solution seeding.
- A deterministic startup seed for singular cases near the home pose.
- Near-axis tolerance handling for vertical lowering from home.

### Constants used

The module uses the simulation values:

- base height: `65 mm`
- link 1: `101 mm`
- link 2: `95 mm`
- link 3: `168 mm`
- yaw limit: `+/-120 deg`
- shoulder limit: `+/-90 deg`
- elbow limits: `+/-120 deg`

### Solver method used

The implemented solver is damped least squares in the yaw-aligned pitch plane.

The flow is:

1. Compute yaw from `atan2(y, x)`.
2. Reduce XYZ to planar `r/z` coordinates.
3. Solve the three pitch joints iteratively using a `2x3` Jacobian.
4. Clamp per-joint pitch angles each iteration.
5. Cap per-iteration step size.
6. Return success only if the final error is acceptable.

### Important improvement added during validation

During host-side smoke testing, a realistic issue appeared: starting at the straight-up home pose is singular for some downward or forward/down targets if the solver starts from all-zero pitch angles.

To address that, the implementation now adds:

- a closed-form two-link pitch seed when starting from the vertical home pose and moving away from full extension,
- near-axis tolerance acceptance so vertical lowering from home can succeed even if the final pose has a small radial offset.

This was an intentional practical decision to make the Z slider usable from home without needing orientation control.

### Documentation added

The in-depth math and implementation README requested by the user was added here:

- `src/arm/src/kinematics/README.md`

---

## 3. Firmware Control Loop Integration

### New mode state

`src/arm/main.c` now owns:

- current control mode,
- current Cartesian target,
- last valid Cartesian IK solution,
- a flag indicating whether the IK layer is currently holding after failure.

### Mode behavior

#### Joint mode

Joint mode behaves like the old system:

- `FFF3` writes all six logical joint targets.

#### Cartesian mode

Cartesian mode behaves as follows:

- `FFF4` writes XYZ targets.
- firmware solves IK for the 4-DOF positioning chain.
- solved sim joints are mapped to real Joints 6/5/4/3.
- real Joints 1 and 2 still come from `FFF3` manual joint packets.

### Important event-processing behavior

`process_ble_events()` was rewritten so that:

- high-rate joint and Cartesian target events are coalesced,
- mode changes preserve FIFO ordering relative to other event types,
- safe pose clears pending targets at the moment it occurs,
- later events after the safe pose can still apply normally.

This matters because mode changes and safe-pose events are control events, not just data events.

### Safe pose behavior

Safe pose now does all of the following:

- zeros all six logical joint targets,
- marks outputs dirty,
- resets the firmware mode to joint mode,
- resets Cartesian solver state.

This is intentionally conservative.

### Seeding on mode switch

When entering Cartesian mode, firmware seeds the Cartesian state from the current joint target registers using forward kinematics.

This avoids an immediate pose jump when the mode switch happens before the first XYZ packet arrives.

---

## 4. macOS BLE Layer Changes

Implemented in `src/mac-app/src/BLEManager.swift`.

### What changed

- Added `ArmControlMode` enum.
- Added published `controlMode` state.
- Added discovery for `FFF4`.
- Added pending Cartesian target buffering.
- Added coalesced Cartesian writes using the existing ~60 Hz timer model.
- Added `setControlMode(_:)` which writes `[0x03, mode]` to `FFF1`.
- Updated TAKE_CONTROL flow so it:
	- takes control,
	- writes the selected mode,
	- then flushes the active target packet.

### Compatibility behavior retained

- Joint target packet encoding remains unchanged.
- Backpressure handling still uses CoreBluetooth capabilities.
- Non-owner writes are still avoided by the app, while firmware still enforces ownership.

---

## 5. macOS UI Changes

Implemented in `src/mac-app/src/ContentView.swift`.

### Joint mode UI

Still shows the original six joint sliders.

### Cartesian mode UI

Now shows:

- X slider
- Y slider
- Z slider
- Joint 1 slider
- Joint 2 slider

### Mode switching UI

Added:

- `Cartesian Control` button when in joint mode
- `Joint Control` button when in Cartesian mode

### Center Pose behavior

Made mode-aware:

- in joint mode: sets all six joints to zero,
- in Cartesian mode: sets XYZ to `(0, 0, 429)` and wrist/gripper to zero.

### UI continuity helper

Added `src/mac-app/src/ArmKinematics.swift`.

This is not meant to replace firmware authority. Its role is only to keep local UI state coherent by:

- computing XYZ from the current joint state when entering Cartesian mode,
- estimating resulting joint angles from XYZ while the user drags Cartesian sliders,
- preventing the UI from reverting to stale joint values when switching back.

---

## Documentation Updated

### Firmware reference

Updated `src/arm/README.md` to describe:

- the new dual-mode architecture,
- the kinematics layer,
- the `FFF4` packet,
- the mode command,
- ownership semantics for Cartesian mode,
- troubleshooting additions,
- file map additions.

### BLE protocol reference

Updated `src/arm/docs/BLE_README.md` to describe:

- `FFF4`,
- `SET_CONTROL_MODE`,
- Cartesian target handling,
- owner-only mode behavior,
- firmware IK behavior.

### macOS app reference

Updated `src/mac-app/README.md` to describe:

- the new mode switch,
- the UI structure,
- the additional characteristic,
- packet formats and units,
- Cartesian-mode behavior.

---

## Validation Performed During This Session

This machine did not have the required full toolchains installed, so validation was best-effort only.

### What was successfully validated

#### Editor diagnostics

VS Code diagnostics reported no errors for:

- `src/arm/src/ble/arm_ble_interface.h`
- `src/arm/src/ble/arm_ble_interface.c`
- `src/arm/src/kinematics/arm_kinematics.h`
- `src/arm/src/kinematics/arm_kinematics.c`
- `src/mac-app/src/BLEManager.swift`
- `src/mac-app/src/ContentView.swift`
- `src/mac-app/src/ArmKinematics.swift`

`src/arm/main.c` only reported the expected missing Pico SDK include-path issue because local firmware tooling was incomplete.

#### Host-side C compile

This command succeeded:

```bash
gcc -std=c11 -Wall -Wextra -Werror -I src/arm/src -c src/arm/src/kinematics/arm_kinematics.c -o /tmp/arm_kinematics.o
```

That confirms the standalone firmware kinematics module is at least syntactically valid and warning-clean under a host compiler.

#### Throwaway IK smoke test

A temporary host-compiled test program was run against `arm_kinematics.c`.

Observed results after refinement:

- home target `(0, 0, 429)` -> success
- target `(120, 0, 250)` -> success
- target `(120, 80, 250)` -> success
- target `(0, 0, 300)` -> success, with a small acceptable radial offset near the vertical axis

This testing directly influenced the addition of the closed-form seed and near-axis tolerance behavior.

### What could not be fully validated here

#### Firmware build

This configure attempt failed because the machine did not have the Pico compiler or make program:

```bash
cmake -S src/arm -B src/arm/build -DPICO_BOARD=pico_w
```

The specific blockers were:

- missing `arm-none-eabi-gcc`
- missing build program for `Unix Makefiles`

#### macOS app build

This build attempt failed because `swift` was not installed on this Linux machine:

```bash
cd src/mac-app && swift build
```

So the Swift changes are editor-checked but not compiler-checked on this machine.

---

## Important Known Limitations / Risks After This Session

These are the most important things the next session should be aware of.

### 1. Firmware and app have not yet been tested together on hardware

The full BLE interaction path has not been exercised yet on a machine with:

- Pico build tooling,
- Swift toolchain,
- and actual BLE-capable hardware.

The biggest remaining risk is integration behavior, not syntax.

### 2. Swift helper kinematics is intentionally approximate

`src/mac-app/src/ArmKinematics.swift` is only for UI continuity. It should not be treated as the source of truth for robot motion. The firmware remains authoritative.

If the app-side estimated joints differ slightly from actual hardware motion, that is acceptable as long as mode switching feels coherent and not misleading.

### 3. Near-axis Cartesian lowering is accepted with a small radial offset

This was a deliberate practical choice so the Z slider can lower from home. If exact axis-preserving vertical behavior is later required, that would need a different kinematic objective or an explicit posture bias.

### 4. No real hardware calibration was added

The implementation uses simulation constants and conventions directly. If the physical arm differs from the simulation in link lengths, offsets, or zero positions, some Cartesian targets may be biased.

That would be a calibration issue, not necessarily a logic bug.

---

## What Still Needs To Be Done

This section is the main actionable continuation plan for the next session.

## 1. Build firmware on the correct machine

Run:

```bash
cmake -S src/arm -B src/arm/build -DPICO_BOARD=pico_w
cmake --build src/arm/build
```

Look for:

- compile errors in `main.c` integration,
- generated GATT handle names for `FFF4`,
- any missing include or linkage issues.

Potential likely area to inspect if build fails:

- generated `arm_control_service.h` handle names for `FFF4`
- C/C++ linkage mismatches
- missing `math` linkage beyond the added `m`

## 2. Build the macOS app

Run:

```bash
cd src/mac-app
swift build
```

Likely things to verify/fix if it fails:

- Swift type inference around closures and bindings
- CoreBluetooth delegate behavior
- whether the package picks up the new `ArmKinematics.swift` file automatically as expected

## 3. Flash firmware and do BLE integration testing

Test the following in order:

1. Connect from the mac app and confirm all four characteristics are discovered.
2. Confirm TAKE_CONTROL still works.
3. Confirm joint mode still drives all six joints exactly as before.
4. Press `Cartesian Control` and confirm the mode write succeeds without disconnecting.
5. Drag X/Y/Z sliders and confirm the arm tip moves in the expected simulation directions.
6. Drag Joint 1 and Joint 2 in Cartesian mode and confirm they still work independently.
7. Confirm RELEASE and disconnect still return to safe pose and joint mode.

## 4. Verify expected coordinate directions on real hardware

Specifically verify:

- increasing X moves the tip forward in the simulation's +X direction,
- changing Y changes yaw in the expected right-hand-rule direction,
- increasing Z moves the tip upward,
- the mapping to real Joints 6/5/4/3 behaves correctly.

If a direction is inverted on real hardware, the likely fix point is not BLE and not the kinematics equations. The likely fix point is the joint transform or mapping logic in `main.c`.

## 5. Check for calibration drift

If the arm reaches generally correct directions but wrong positions, investigate:

- physical link lengths,
- zero offsets,
- whether simulation constants need calibration overrides.

Do not immediately change the BLE packet contract or the mac UI for this.

## 6. Decide whether app-side joint estimation should be refined

If switching back from Cartesian mode to joint mode feels visually inconsistent in the app, refine `src/mac-app/src/ArmKinematics.swift`.

This is lower priority than hardware verification because the firmware is the true control path.

---

## Suggested Continuation Prompt For The Next Session

If starting a new session on the other machine, a good prompt would be:

> Continue the Cartesian XYZ control mode implementation in this repo. Read `Session_summary.md` first. The feature was implemented but not fully built on the previous machine because the Pico and Swift toolchains were missing. Build the firmware and macOS app, fix any compile or integration issues, then verify the BLE mode switch, Cartesian XYZ motion, and wrist/gripper behavior on hardware.

---

## Current Repo State Notes

At the end of this session, `git status --short` showed:

- modified feature files in firmware and mac app,
- new kinematics directory under `src/arm/src/kinematics/`,
- new `src/mac-app/src/ArmKinematics.swift`,
- unrelated existing worktree changes:
	- deleted `kinematic_prompt.md`
	- new `.python-version`

Those unrelated changes were not reverted.

---

## Bottom Line

This session completed the implementation pass for Cartesian XYZ control across firmware, BLE, macOS UI, and documentation.

The feature is structurally in place.

What remains is the machine-specific part:

- real firmware build,
- real Swift build,
- real hardware/BLE integration validation,
- and any follow-up fixes discovered from those builds or tests.

The most important technical decisions are already made and documented here, so the next session should focus on build/test/fix rather than redesign.