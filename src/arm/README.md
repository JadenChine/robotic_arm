# Arm Firmware Reference

## Purpose

This document is the primary technical reference for the Pico firmware in src/arm.

It explains:

- what this firmware controls,
- how the protocol backends are structured,
- how to build/flash/test,
- and where to make changes safely.

The goal is to keep this file evergreen: accurate, practical, and focused on implementation reality.

## What This Firmware Does

The firmware provides a C HAL and an interactive smoke test for controlling bus servos from a Raspberry Pi Pico.

Two UART protocol backends are supported:

1. Direct smart-servo protocol
2. xArm controller-board protocol

Current default behavior targets xArm controller boards (for example xArm v1.6):

- backend: controller mode
- bus baud rate: 9600
- Pico UART pins: GP0 TX, GP1 RX

## Current Verified Setup

The currently validated hardware path is:

- Pico UART -> xArm bus-servo controller board UART
- controller board -> daisy-chained servos

Known-good usage includes selecting and controlling any of six servos in the smoke test, including selecting servo ID 1 (end of chain).

## Architecture

### 1) Platform Layer

Path: src/arm/src/platform

Responsibilities:

- Pico UART initialization and I/O
- timing source access (millis)
- hardware default values (UART index, pins, baud)

Key files:

- src/arm/src/platform/pico_uart_transport.hpp
- src/arm/src/platform/pico_uart_transport.cpp
- src/arm/src/platform/servo_platform_config.h

### 2) Driver Layer

Path: src/arm/src/drivers

Responsibilities:

- packet encode/decode
- command/response parsing
- protocol-specific command mapping

Implemented drivers:

1. Direct smart-servo driver
- files:
  - src/arm/src/drivers/servo_bus_driver.hpp
  - src/arm/src/drivers/servo_bus_driver.cpp
- framing:
  - 0x55 0x55 ID LEN CMD DATA... CHECKSUM
- target:
  - direct servo bus communication

2. xArm controller driver
- files:
  - src/arm/src/drivers/xarm_controller_driver.hpp
  - src/arm/src/drivers/xarm_controller_driver.cpp
- framing:
  - 0x55 0x55 LEN CMD PARAMS...
- no checksum byte
- target:
  - Pico connected to controller board UART

### 3) HAL Layer

Path: src/arm/src/hal

Responsibilities:

- stable C API for arm application code
- backend-agnostic entry points
- backend routing and status mapping

Key files:

- src/arm/src/hal/servo_bus.h
- src/arm/src/hal/servo_bus.cpp

Important HAL concepts:

- backend selection via servo_bus_backend_t
- config object includes backend + UART settings
- unsupported operations return SERVO_BUS_STATUS_ERROR_UNSUPPORTED

### 4) Application Layer

Path: src/arm/main.c

Responsibilities:

- heartbeat LED liveness loop
- USB serial command interface for bench validation
- quick protocol and wiring validation without higher-level planners

## Backend Comparison

### Direct Servo Backend

Use when Pico talks to servo bus directly (with appropriate electrical interface).

- default direct baud: 115200
- supports rich per-servo smart-servo features
- packet includes ID and checksum

### Controller Backend

Use when Pico talks to a HiWonder/xArm controller board that then drives servos.

- default controller baud: 9600
- packet excludes dedicated ID field and checksum
- command set matches controller protocol documentation
- some direct-servo features are intentionally unsupported in this mode

## Default Configuration

Defaults are defined in src/arm/src/platform/servo_platform_config.h.

Current defaults:

- SERVO_BUS_DEFAULT_BACKEND = controller mode
- SERVO_BUS_DEFAULT_UART_INDEX = 0
- SERVO_BUS_DEFAULT_TX_PIN = 0 (GP0)
- SERVO_BUS_DEFAULT_RX_PIN = 1 (GP1)
- SERVO_BUS_CONTROLLER_DEFAULT_BAUD_RATE = 9600
- SERVO_BUS_DIRECT_DEFAULT_BAUD_RATE = 115200

## Smoke Test Console (main.c)

The smoke test is intentionally keyboard-driven and minimal.

Command set:

- h or ? : show help
- 1..6 : select active servo ID
- p : ping active servo
- c : move active servo to center
- a : step left
- d : step right
- s : stop/unload active servo
- r : read active servo position
- v : voltage read
  - controller mode: controller battery voltage
  - direct mode: servo voltage
- t : temperature read
  - controller mode: unsupported (clean status)
  - direct mode: servo temperature
- e : print last driver error

Behavior notes:

- active servo selection persists until changed
- each servo keeps its own tracked target angle for stepping behavior
- heartbeat LED continues regardless of command success/failure

## Build and Flash

Project root for firmware:

- src/arm

Typical flow:

1. Configure (first time):

```bash
cmake -S src/arm -B src/arm/build
```

2. Build and flash:

```bash
cd src/arm
./scripts/build_and_flash.sh
```

3. Build, flash, and auto-connect serial monitor:

```bash
cd src/arm
./scripts/build_and_flash.sh --connect
```

Notes:

- scripts/build_and_flash.sh expects picotool
- scripts/serial_connect.sh expects picocom

## Wiring Guidance

For controller backend (default):

- Pico GP0 (TX) -> controller RX
- Pico GP1 (RX) -> controller TX
- Pico GND -> controller GND

Also verify:

- voltage-level compatibility on controller TX line into Pico RX
- shared ground is present
- controller and servo power are stable

## File Map

Top-level firmware files and folders:

- src/arm/CMakeLists.txt
- src/arm/main.c
- src/arm/scripts/build_and_flash.sh
- src/arm/scripts/serial_connect.sh
- src/arm/src/platform/
- src/arm/src/drivers/
- src/arm/src/hal/

## Design Rules for Future Changes

1. Keep protocol parsing in drivers, not in HAL or main.c.
2. Keep HAL C-facing and backend-neutral.
3. Preserve both backends unless intentionally removed.
4. Add new protocol features to the correct driver first, then map through HAL.
5. Keep smoke test simple and focused on bring-up/diagnostics.

## Known Limitations

In controller backend mode:

- not all direct-servo telemetry/control functions exist
- unsupported HAL calls return SERVO_BUS_STATUS_ERROR_UNSUPPORTED by design

This is expected behavior, not a failure.

## Quick Troubleshooting

If servos do not respond:

1. Confirm backend is controller mode when using xArm controller board.
2. Confirm bus baud is 9600 for controller mode.
3. Confirm GP0/GP1 wiring and shared GND.
4. Use the smoke test sequence:
   - select servo ID (1..6)
   - ping (p)
   - read position (r)
   - move center (c)
5. Check last driver error with e.

If serial monitor works but commands fail:

- protocol mismatch is likely (wrong backend or wrong baud)

If only some IDs move:

- verify each servo ID assignment and chain integrity

## Maintenance Checklist

When updating firmware behavior, also update this README:

- protocol/backend assumptions
- default pins/baud/backend
- smoke-test command list
- limitations and expected unsupported paths
- any new files introduced in platform/drivers/hal/app
