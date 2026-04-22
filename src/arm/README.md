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

The firmware provides a C HAL and an application-layer control loop for commanding six arm joints from a Raspberry Pi Pico.

Two UART protocol backends are supported:

1. Direct smart-servo protocol
2. xArm controller-board protocol

Current default behavior targets xArm controller boards (for example xArm v1.6):

- backend: controller mode
- bus baud rate: 9600
- Pico UART pins: GP0 TX, GP1 RX

In addition, the firmware now exposes a BLE GATT server on Pico W hardware:

- BLE role: peripheral
- intended centrals: Mac controller app and/or remote Pico W controller
- BLE stack execution: core1
- servo control loop + UART traffic: core0

## Current Verified Setup

The currently validated hardware path is:

- Pico UART -> xArm bus-servo controller board UART
- controller board -> daisy-chained servos

Known-good usage includes controlling any of six servos through the controller backend, including servo ID 1 (end of chain).

## Architecture

### 1) Platform Layer

Path: src/arm/src/platform

Responsibilities:

- Pico UART initialization and I/O
- timing primitives for periodic scheduling
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
- six global joint target registers (millidegrees)
- repeating timer interrupt that schedules control ticks
- periodic dirty-setpoint transmission through HAL

### 5) BLE Control Layer

Path: src/arm/src/ble

Responsibilities:

- expose BLE GATT service for remote target writes
- enforce single-owner control token semantics
- queue cross-core BLE events for core0 application loop
- trigger safe-pose fallback on owner disconnect/timeout

Key files:

- src/arm/src/ble/arm_ble_interface.h
- src/arm/src/ble/arm_ble_interface.c
- src/arm/src/ble/arm_control_service.gatt
- src/arm/src/btstack_config.h

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

## Joint Target Control Loop (main.c)

The application now runs as a periodic controller rather than a keyboard-driven smoke test.

### Target Registers

- `g_joint_target_mdeg[0..5]` holds desired joint angles in millidegrees.
- Register index maps to one arm joint.
- Current default per-joint software limits are -90 deg to +90 deg.

### Timing Model

- A repeating timer interrupt is configured from `CONTROL_TICK_HZ`.
- The IRQ callback is intentionally minimal and only marks a control tick as pending.
- UART transmission is executed in the main loop, not in IRQ context.

### Tick Behavior

On each scheduled control tick, the application:

1. snapshots all 6 target registers,
2. applies per-joint clamp, direction, and offset,
3. converts millidegrees to protocol raw position (0..1000),
4. applies a deadband against last-sent raw values,
5. sends only dirty joints in one `servo_bus_move_multiple(...)` command.

Startup behavior:

- all target registers initialize to 0 deg,
- first tick forces an initial command to establish a known pose.

Current app-level defaults in `main.c`:

- `CONTROL_TICK_HZ = 20`
- `CONTROL_MOVE_TIME_MS = 120`
- `CONTROL_RAW_DEADBAND = 2`

These values are intended as a stable starting point for future human-in-the-loop control (encoders/sliders).

## Build and Flash

Project root for firmware:

- src/arm

Typical flow:

1. Configure (first time):

```bash
cmake -S src/arm -B src/arm/build -DPICO_BOARD=pico_w
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
- BLE requires Pico W (CYW43)

## BLE Control Interface

### GATT Service

Custom service UUIDs (Bluetooth base form):

- service: `0xFFF0`
- control point: `0xFFF1` (write)
- owner token: `0xFFF2` (read, dynamic)
- joint targets: `0xFFF3` (write/write-without-response)

### Control Commands (`0xFFF1`)

- `0x01`: take control (steals immediately)
- `0x02`: release control

### Owner Token (`0xFFF2`)

- 4-byte little-endian `uint32`
- `0` means no active owner
- non-zero value changes whenever ownership is granted

### Joint Target Packet (`0xFFF3`)

Binary payload (12 bytes total):

- 6 x `int16` little-endian
- unit: deci-degrees (`0.1 deg`)
- order: joint1, joint2, joint3, joint4, joint5, joint6

Firmware conversion path:

1. deci-degrees -> millidegrees (`value * 100`)
2. write to `g_joint_target_mdeg[0..5]`
3. existing 20 Hz control loop applies clamp/transform/deadband and transmits via UART backend

### Ownership Rules

- up to 2 BLE centrals can connect simultaneously
- only current owner can write joint targets
- non-owner target writes are rejected by GATT
- if owner disconnects or stops sending updates for timeout window (`BLE_OWNER_TIMEOUT_MS`), ownership clears and safe pose (all zero targets) is commanded

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
- src/arm/src/ble/
- src/arm/src/btstack_config.h

## Design Rules for Future Changes

1. Keep protocol parsing in drivers, not in HAL or main.c.
2. Keep HAL C-facing and backend-neutral.
3. Preserve both backends unless intentionally removed.
4. Add new protocol features to the correct driver first, then map through HAL.
5. Keep IRQ callbacks short and non-blocking; do serial TX in non-IRQ code.

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
4. Confirm firmware prints successful servo bus initialization.
5. Confirm control timer started message appears at boot.

If BLE clients cannot control motion:

1. Confirm board is Pico W and build used `-DPICO_BOARD=pico_w`.
2. Confirm central writes `0x01` to `0xFFF1` before slider packets.
3. Confirm joint packets are exactly 12 bytes (`6 x int16`, little-endian, 0.1 deg units).
4. Confirm active owner has not timed out.

If movement is jerky or bus-heavy:

1. Lower `CONTROL_TICK_HZ`.
2. Increase `CONTROL_RAW_DEADBAND`.
3. Increase `CONTROL_MOVE_TIME_MS` for smoother interpolation.

If serial monitor works but commands fail:

- protocol mismatch is likely (wrong backend or wrong baud)

If only some IDs move:

- verify each servo ID assignment and chain integrity

If the arm does not react to changing target registers:

- verify updates are writing the intended index in `g_joint_target_mdeg[0..5]`
- verify values remain within configured software limits

## Maintenance Checklist

When updating firmware behavior, also update this README:

- protocol/backend assumptions
- default pins/baud/backend
- control loop rate/deadband/move-time defaults
- limitations and expected unsupported paths
- any new files introduced in platform/drivers/hal/app
