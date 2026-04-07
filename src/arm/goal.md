# Arm Servo Driver Bootstrap Goal

## Purpose

This file is the persistent continuity document for the low-level servo control bootstrap work in `src/arm/`.

Its job is to preserve the important project context, design decisions, implementation plan, execution notes, and environment blockers so the work can continue in a later chat session or by another contributor without depending on chat history.

This file is intentionally practical. It should capture actionable rationale and project decisions, not hidden internal reasoning.

## User Intent

- Port the HiWonder intelligent bus servo functionality into the robotic arm firmware under `src/arm/`.
- Do not keep `src/HiBusServo/` as a long-term dependency in the arm firmware. That repository is reference material only.
- Refactor the serial integration so it works cleanly on Raspberry Pi Pico using Pico SDK.
- Overwrite the existing `src/arm/main.c`, but keep the blinking indicator so firmware liveness remains obvious.
- Provide a simple first-stage interface that lets a host computer keyboard control one connected servo with ID `1`.
- Bring over the full low-level functionality for future use, even though the first demo only exercises a small subset.
- Structure the result so it becomes the base of a hardware abstraction layer for a larger robotic arm controller.

## What Was Determined In Planning

### Chosen architecture

The implementation should use three layers:

1. `platform/`
	 Owns Pico-specific UART and timing integration.
2. `drivers/`
	 Owns the HiWonder packet protocol and full servo feature set.
3. `hal/`
	 Exposes a stable C-facing API for the rest of the arm firmware.

This keeps the protocol implementation reusable and keeps future arm logic from depending on packet details or C++ types.

### Language boundary

- Internal driver code may remain C++ where it is the most direct fit.
- Public arm-facing HAL should remain C-friendly.
- `main.c` should stay in C and talk only to the HAL.

### Demo scope

The first keyboard-driven smoke test should stay intentionally small:

- print help
- ping servo `1`
- move to center
- step left
- step right
- stop motion
- read position
- read voltage
- read temperature
- print last error

This is enough to prove the bus path, command loop, and telemetry path without prematurely designing a full motion console.

### Hardware direction

- USB stdio is used for the host keyboard console.
- A dedicated hardware UART is used for the servo bus.
- The initial firmware should keep the current heartbeat LED behavior while expanding into an interactive test application.
- Default UART wiring should be centralized in one config header so rewiring only requires one edit.

## Reference Material

These files were inspected and are the behavior reference for the port:

- `src/HiBusServo/README.md`
- `src/HiBusServo/src/HiBusServo.h`
- `src/HiBusServo/src/HiBusServo.cpp`

Important facts from the reference library:

- Packet format is `0x55 0x55 ID LENGTH COMMAND DATA... CHECKSUM`.
- The implementation is synchronous and blocking.
- Response timeout is `20 ms`.
- The original code depends on Arduino concepts such as `HardwareSerial`, `Arduino.h`, `millis()`, `constrain()`, `lowByte()`, `highByte()`, and `word()`.
- The reference code already includes more than the minimum demo features, including movement, wait moves, broadcast operations, ID management, telemetry, limits, mode switching, load control, and LED control.

## Scope

### In scope

- Copy the reference driver behavior into arm-owned source files.
- Remove Arduino dependencies.
- Add Pico UART transport.
- Add a C HAL boundary.
- Replace `main.c` with a heartbeat-plus-console smoke test.
- Update Linux host serial tooling.
- Document build, flash, and test flow.
- Keep this file updated with progress and blockers.

### Out of scope for this bootstrap

- inverse kinematics
- multi-joint coordinated motion
- wireless transport
- web/mobile interface integration
- the higher-level unified command protocol described in `Design_Doc.md`

## Implementation Structure

Recommended arm-owned layout:

- `src/arm/src/platform/`
	Pico UART transport and hardware defaults.
- `src/arm/src/drivers/`
	Internal servo protocol implementation.
- `src/arm/src/hal/`
	Stable C-facing servo API.
- `src/arm/main.c`
	Minimal interactive smoke test and heartbeat loop.

## Execution Order

1. Initialize the Pico SDK submodule and establish the build baseline.
2. Create this continuity file and keep it current.
3. Scaffold the platform, driver, and HAL directories.
4. Port the reference servo driver into arm-owned files.
5. Implement the Pico UART transport.
6. Expose a C-facing HAL.
7. Update CMake for the mixed C/C++ build.
8. Replace `main.c` with the console-driven smoke test.
9. Update serial tooling and README instructions.
10. Validate build, firmware startup, LED heartbeat, servo control, telemetry, and timeout behavior.

## Verification Requirements

- Firmware should build once a valid Pico toolchain is installed.
- LED heartbeat must remain active.
- USB serial console must accept host keyboard input.
- Servo ID `1` must respond to ping and move commands.
- Position, voltage, and temperature reads must work when hardware is connected.
- Timeout and disconnect cases must not hang the main loop.
- The public HAL must remain usable from C without exposing C++ types.
- This file must remain sufficient for a future session to resume the work.

## Environment Findings And Blockers

### Repository state at implementation start

- Branch in use: `feature/initial_servo_control`
- `src/HiBusServo/` is currently present as reference material and is untracked.
- `src/arm/goal.md` existed but was empty.

### Build environment findings

- `src/pico-sdk/` was initially empty.
- The Pico SDK submodule has now been initialized.
- The current machine does not have a working Pico cross-compilation environment yet.

Observed blocking issues during baseline build:

- `arm-none-eabi-gcc` not found
- build generator not configured on this machine during CMake configure

Implication:

Code implementation can proceed, but final compile validation must wait until a Pico toolchain and generator are installed or configured.

## Decisions Made So Far

- Keep `src/HiBusServo/` as reference material only.
- Preserve the full low-level servo feature set in the imported arm-owned driver.
- Use a pure C HAL over an internal Pico-adapted C++ driver.
- Keep USB stdio for the operator console and hardware UART for the servo bus.
- Keep the heartbeat LED in the new `main.c`.
- Start with a narrow smoke test, not a complex command shell.

## Resume Instructions

If work resumes in a later session, start here:

1. Read this file first.
2. Check the current implementation files under `src/arm/src/`.
3. Verify whether the Pico toolchain has been installed.
4. Build the firmware before changing behavior if the toolchain is available.
5. Continue the next incomplete item from the execution log below.

## Execution Log

### 2026-04-07

- Explored `src/HiBusServo/` to understand the driver architecture and serial assumptions.
- Explored `src/arm/` to confirm the project is a Pico SDK firmware with a blink-only `main.c`.
- Determined the correct long-term boundary is a C HAL over an internal Pico-adapted driver.
- Decided the first demo should use USB serial keyboard control for servo ID `1`.
- Initialized the `src/pico-sdk` submodule.
- Attempted a baseline build and confirmed the environment is blocked by missing `arm-none-eabi-gcc` and an incomplete local build-tool setup.
- Added arm-owned platform defaults, Pico UART transport, internal servo bus driver, and a broad C-facing HAL under `src/arm/src/`.
- Replaced `src/arm/main.c` with a heartbeat-driven USB serial smoke test for servo ID `1`.
- Updated the host serial helper to search Linux device paths in addition to macOS-style names.
- Updated the project README with the initial servo bootstrap workflow and default hardware assumptions.
- Full firmware build and hardware validation remain blocked on the missing Pico cross-toolchain in the current environment.

## Next Intended Steps

- Install or point CMake to a working Pico toolchain with `arm-none-eabi-gcc` and a generator such as `ninja`.
- Configure and build `src/arm/` on a machine with that toolchain available.
- Flash the Pico and verify USB serial console startup, heartbeat LED behavior, and command handling.
- Connect the HiWonder bus controller and validate ping, movement, position, voltage, and temperature reads against servo ID `1`.
- Adjust UART defaults in `src/arm/src/platform/servo_platform_config.h` if the final board wiring differs from the current default assumptions.
