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

It supports two owner-gated control modes:

1. Joint target mode: BLE writes six logical joint angles directly.
2. Cartesian XYZ mode: BLE writes an end-effector tip target in millimeters, then firmware IK drives the four positioning joints while wrist/gripper joints remain manually controlled.

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
- active-HIGH TX/RX buffer enables and bus turnaround for direct servos
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
- six global joint target registers (deci-degrees, 0.1°)
- joint-target and Cartesian control-mode state
- repeating timer interrupt that schedules control ticks
- periodic dirty-setpoint transmission through HAL

### 5) Kinematics Layer

Path: src/arm/src/kinematics

Responsibilities:

- forward kinematics for the simulation-compatible 4-DOF positioning chain
- damped least-squares inverse kinematics for Cartesian XYZ targets
- simulation-frame link lengths, base offset, signs, and joint limits
- Cartesian target clamping and hold-last-valid behavior for failed IK solves

Key files:

- src/arm/src/kinematics/arm_kinematics.h
- src/arm/src/kinematics/arm_kinematics.c
- src/arm/src/kinematics/README.md

### 6) BLE Control Layer

Path: src/arm/src/ble

Responsibilities:

- expose BLE GATT service for remote target writes
- enforce single-owner control semantics
- accept owner-only control-mode switches
- queue cross-core BLE events for core0 application loop
- trigger safe-pose fallback on owner release/disconnect

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

Direct protocol command behavior:

- LED control follows the protocol's polarity: `0` means on and `1` means off,
  including broadcast LED commands.
- ID-read requests to broadcast ID `254` accept a reply carrying the responding
  servo's actual ID. Use discovery with only one servo connected. Other broadcast
  reads return `SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT` without transmitting,
  because the protocol specifies that they do not return replies.
- Direct movement times are clamped to `0..30000 ms`, including queued and
  broadcast moves. Existing raw-position clamping and centered `-120..120`
  degree coordinates are preserved.
- New servo IDs must be `0..253`; packet destination `255` is invalid. LED alarm
  masks must be `0..7`. Angle and voltage limits retain their existing range
  clamping, then require minimum strictly below maximum. Invalid requests return
  `SERVO_BUS_STATUS_ERROR_INVALID_ARGUMENT` without sending a packet. The direct
  driver's corresponding last-error code is `kServoErrorInvalidArgument` (`-3`).
- A multi-servo move validates all IDs before queuing any move or sending START.
- The optional reads for last commanded position/time (command `2`) and LED state
  (command `34`) remain unexposed; current position and queued position/time reads
  are supported. Torque startup behavior remains under application control.

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
- SERVO_BUS_DIRECT_DEFAULT_TX_ENABLE_PIN = 2 (GP2, physical pin 4)
- SERVO_BUS_DIRECT_DEFAULT_RX_ENABLE_PIN = 3 (GP3, physical pin 5)
- SERVO_BUS_CONTROLLER_DEFAULT_BAUD_RATE = 9600
- SERVO_BUS_DIRECT_DEFAULT_BAUD_RATE = 115200

The enable pins are used only by the direct backend. A fresh controller-backend
initialization leaves them untouched. Reinitializing from direct to controller
mode first disables both old buffers and leaves their enable pins as inputs with
pull-downs.

To select direct mode explicitly, start with the default config and override both
the backend and baud before calling `servo_bus_init`:

```c
servo_bus_config_t config;
servo_bus_get_default_config(&config);
config.backend = SERVO_BUS_BACKEND_DIRECT_SERVO;
config.baud_rate = 0; // Select the backend's default: 115200 for direct mode.
// config.tx_enable_pin = 2; // Already populated by get_default_config.
// config.rx_enable_pin = 3;
servo_bus_status_t status = servo_bus_init(&config);
```

Changing only `config.backend` after fetching the controller defaults would retain
9600 baud. The checked-in application continues to select the controller backend.

### Direct bus direction and timing

The transport uses ordinary, non-inverted UART0 on GP0/GP1, **115200 baud, 8N1**,
with hardware flow control disabled. GP2 and GP3 are ordinary GPIO outputs driving
the two buffers' active-HIGH OE inputs; UART RTS/CTS are not used.

| State | GP2 / TX_EN | GP3 / RX_EN |
|---|---|---|
| UART initialization | LOW | LOW |
| Listening (including after a timeout) | LOW | HIGH |
| Sending one packet | HIGH | LOW |

Initialization keeps both buffers disabled until UART is ready and TX idles HIGH.
The Pico RX pad has an internal pull-up so it stays HIGH when the receive buffer
is disabled. For each direct packet, the driver clears stale RX bytes before
sending. The transport then disables RX, enables TX, allows 1 microsecond for OE
propagation, sends the packet, and waits for the UART **BUSY flag to clear**. This
wait includes the shift register and final stop bit, not just an empty FIFO.
It then disables TX and immediately enables RX before returning from `write()`.
No RX bytes are discarded after this turnaround, so an immediate reply survives
the driver's subsequent `flush()` call. Commands without replies and each packet
in a multi-servo move also return to listening.

Core0 interrupts are masked from entering transmit mode through returning to
listening, so an ISR cannot hold the bus driven after transmission completes.
Current direct command packets are at most 10 bytes: about 0.87 ms at 115200 baud,
plus small software overhead. BLE runs on core1. Keep all HAL/UART calls serialized
on core0, outside IRQ handlers, as the current application does. Longer packets or
slower custom baud rates would extend this interrupt-masking interval.

## Joint Target Control Loop (main.c)

The application now runs as a periodic controller rather than a keyboard-driven smoke test.

### Target Registers

- `g_joint_target_deci_deg[0..5]` holds desired joint angles in deci-degrees (0.1° units).
- Register index maps to one arm joint.
- Current per-joint software limits (degrees):
  - Joint 1: -120.0° to +10.0°
  - Joint 2: -120.0° to +120.0°
  - Joint 3: -120.0° to +120.0°
  - Joint 4: -120.0° to +120.0° (direction reversed in software)
  - Joint 5: -90.0° to +90.0°
  - Joint 6: -120.0° to +120.0°

### Timing Model

- A repeating timer interrupt is configured from `CONTROL_TICK_HZ`.
- The IRQ callback is intentionally minimal and only marks a control tick as pending.
- UART transmission is executed in the main loop, not in IRQ context.

### Tick Behavior

On each scheduled control tick, the application:

1. snapshots all 6 target registers,
2. applies per-joint clamp, direction, and offset,
3. converts deci-degrees (0.1°) to protocol raw position (0..1000),
4. applies a deadband against last-sent raw values,
5. sends only dirty joints in one `servo_bus_move_multiple(...)` command.

Startup behavior:

- all target registers initialize to 0 deg,
- first tick forces an initial command to establish a known pose.

Current app-level defaults in `main.c`:

- `CONTROL_TICK_HZ = 20`
- `CONTROL_MOVE_TIME_MS = 90`
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
- owner status: `0xFFF2` (read, dynamic)
- joint targets: `0xFFF3` (write/write-without-response)

### Control Commands (`0xFFF1`)

- `0x01`: take control (steals immediately)
- `0x02`: release control

### Owner Status (`0xFFF2`)

- 1 byte
- `0x01` if the reading connection is currently the owner
- `0x00` otherwise

### Joint Target Packet (`0xFFF3`)

Binary payload (12 bytes total):

- 6 x `int16` little-endian
- unit: deci-degrees (`0.1 deg`)
- order: joint1, joint2, joint3, joint4, joint5, joint6

Firmware conversion path:

1. decode joint targets as deci-degrees (0.1°)
2. in joint mode, write all six values to `g_joint_target_deci_deg[0..5]`
3. in Cartesian mode, write only real Joints 1-2 and ignore Joints 3-6 because IK owns them
4. existing 20 Hz control loop applies clamp/transform/deadband and transmits via UART backend

### Cartesian Target Packet (`0xFFF4`)

Binary payload (6 bytes total):

- 3 x `int16` little-endian
- unit: millimeters
- order: X, Y, Z

Firmware conversion path:

1. decode XYZ as millimeters in the simulation coordinate frame
2. clamp simple workspace bounds (ground, max reach, yaw)
3. solve IK seeded from the last valid Cartesian solution
4. map sim Joint 1/2/3/4 to real Joint 6/5/4/3 logical target registers
5. existing 20 Hz control loop applies clamp/transform/deadband and transmits via UART backend

### Control Mode Command (`0xFFF1`)

Control point commands are:

- `0x01`: take control (steals immediately)
- `0x02`: release control
- `0x03, mode`: set control mode

Mode values:

- `0x00`: joint target mode
- `0x01`: Cartesian XYZ mode

Only the current owner can set mode. Non-owner target writes remain ACK'd as success and ignored.

### Ownership Rules

- up to 2 BLE centrals can connect simultaneously
- only current owner can write joint targets
- only current owner can switch control modes or write Cartesian targets
- non-owner target writes are ignored (ACK'd as success) and do not change targets
- safe pose (all zeros) is commanded on owner release or owner disconnect, and firmware returns to joint mode

## Cartesian Control Mode

Cartesian mode uses the 4-DOF positioning model from `sim/README.md`:

- origin: ground reference at `(0, 0, 0)` mm
- shoulder/base joint: `(0, 0, 65)` mm
- link lengths: `101 mm`, `95 mm`, `168 mm`
- home tip position: `(0, 0, 429)` mm

IK ignores wrist/gripper joints. The mapping is:

- Sim Joint 1 (base yaw) -> real Joint 6
- Sim Joint 2 (shoulder pitch) -> real Joint 5
- Sim Joint 3 (elbow 1) -> real Joint 4
- Sim Joint 4 (elbow 2) -> real Joint 3

Real Joints 1-2 remain controlled by the existing joint target packet in Cartesian mode.

The IK solver is damped least squares in the yaw-aligned pitch plane. It is seeded from the last valid solution to reduce jitter during small slider changes. If a target cannot be solved after workspace clamping, firmware holds the last valid IK pose and continues accepting wrist/gripper updates.

## Wiring Guidance

For controller backend (default):

- Pico GP0 (TX) -> controller RX
- Pico GP1 (RX) -> controller TX
- Pico GND -> controller GND

Also verify:

- voltage-level compatibility on controller TX line into Pico RX
- shared ground is present
- controller and servo power are stable

### Direct servo backend wiring

Use the **5-pin DBV/SOT-23 packages** for the pin numbers below. These are the
single-gate buffers' pin numbers, not the multi-gate 74HC126 numbers in Hiwonder's
diagram.

| Connection | SN74AHCT1G126 (TX, powered at 5 V) | SN74LVC1G126 (RX, powered at 3.3 V) |
|---|---|---|
| Pin 1, OE | Pico GP2, physical pin 4 | Pico GP3, physical pin 5 |
| Pin 2, A | Pico GP0 / UART TX, physical pin 1 | Shared servo data bus |
| Pin 3, GND | Common ground | Common ground |
| Pin 4, Y | Shared servo data bus | Pico GP1 / UART RX, physical pin 2 |
| Pin 5, VCC | Regulated 5 V | Pico 3V3(OUT), physical pin 36 |

The common bus joins TX buffer Y, RX buffer A, and every servo's signal terminal.
Add one 10 kΩ bus pull-up to regulated 5 V, 100 nF supply decoupling at each IC,
and 10 kΩ pull-downs on both OE pins to keep the buffers disabled during reset
before firmware runs. Add the agreed 10 kΩ pull-up from Pico RX to 3.3 V (the
internal pull-up supplements it), and a 10 kΩ pull-up from TX buffer A to 3.3 V.
Pico physical pin 3 is **ground**, not GP2. Connect Pico ground, buffer grounds,
servo ground and motor-supply negative together. Motor power is separate from
the regulated 5 V logic rail. Keep the AHCT 5 V rail powered while the bus is live.

The firmware rejects duplicate OE pins, OE/UART pin collisions, non-header GPIOs,
invalid UART pin mappings, and enable pins overlapping the heartbeat LED. Custom
OE assignments must also avoid any additional hardware connected by the user.
Initialize config through `servo_bus_get_default_config` so the new enable fields
are populated; they are ignored by the controller backend.

Hardware references: [Pico pinout](https://datasheets.raspberrypi.com/pico/Pico-R3-A4-Pinout.pdf),
[TI AHCT datasheet](https://www.ti.com/lit/ds/symlink/sn74ahct1g126.pdf),
[TI LVC datasheet](https://www.ti.com/lit/ds/symlink/sn74lvc1g126.pdf), and the
local [Hiwonder protocol](docs/Hiwonder_Bus_Servo_Communication_Protocol.md).

### Direct bus verification

Host regression tests compile the real transport, HAL and both drivers against
simulated GPIO/UART hardware. From the repository root:

```bash
cmake -S src/arm/tests -B /tmp/robotic-arm-uart-tests
cmake --build /tmp/robotic-arm-uart-tests
ctest --test-dir /tmp/robotic-arm-uart-tests --output-on-failure
```

Tests check safe startup, OE ordering, waiting through the final stop bit,
immediate reply preservation, no-reply/multi-servo commands, timeouts, invalid
pin assignments, backend switching and the controller default/packet format.
Command regression cases also cover LED polarity, discovery replies, invalid
parameters, movement-time limits, and preservation of centered degree coordinates.
These tests and a Pico W firmware build do not establish electrical performance
on the actual servo chain. The controller path remains the hardware-validated
setup; the direct path still needs bench verification.

For direct bring-up, start with one servo with a known ID. Confirm a voltage or
position read through the HAL, then a move, then repeat with the daisy chain.
With a logic analyzer/scope, check GP2 stays HIGH through the final stop bit and
drops before a servo drives its reply, GP3 is LOW during commands and HIGH during
replies, and the bus is idle HIGH. Check edges and levels on the actual cabling,
including servo replies. Use unique servo IDs; broadcast ID reads with multiple
servos cause reply collisions. The normal application sends joint targets soon
after initialization, so account for that when choosing the bench setup.

## File Map

Top-level firmware files and folders:

- src/arm/CMakeLists.txt
- src/arm/main.c
- src/arm/scripts/build_and_flash.sh
- src/arm/scripts/serial_connect.sh
- src/arm/src/platform/
- src/arm/src/drivers/
- src/arm/src/hal/
- src/arm/src/kinematics/
- src/arm/src/ble/
- src/arm/src/btstack_config.h

## Design Rules for Future Changes

1. Keep protocol parsing in drivers, not in HAL or main.c.
2. Keep HAL C-facing and backend-neutral.
3. Preserve both backends unless intentionally removed.
4. Add new protocol features to the correct driver first, then map through HAL.
5. Keep IRQ callbacks short and non-blocking; do serial TX in non-IRQ code.
6. Keep Cartesian math in `src/arm/src/kinematics`, not in BLE handlers or HAL code.

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
4. Confirm Cartesian packets are exactly 6 bytes (`3 x int16`, little-endian, mm units) when using XYZ mode.
5. Confirm UI shows OWNER (owner status reads `0x01`).

If Cartesian mode does not move as expected:

1. Confirm central writes `[0x03, 0x01]` to `0xFFF1` before writing `0xFFF4`.
2. Confirm XYZ values are in millimeters using the simulation coordinate frame.
3. Confirm requested targets are reachable from the shoulder within the 364 mm link reach.
4. Check serial output for `cartesian ik hold`, which means firmware is holding the last valid IK pose.

If movement is jerky or bus-heavy:

1. Lower `CONTROL_TICK_HZ`.
2. Increase `CONTROL_RAW_DEADBAND`.
3. Increase `CONTROL_MOVE_TIME_MS` for smoother interpolation.

If serial monitor works but commands fail:

- protocol mismatch is likely (wrong backend or wrong baud)

If only some IDs move:

- verify each servo ID assignment and chain integrity

If the arm does not react to changing target registers:

- verify updates are writing the intended index in `g_joint_target_deci_deg[0..5]`
- verify values remain within configured software limits

## Maintenance Checklist

When updating firmware behavior, also update this README:

- protocol/backend assumptions
- default pins/baud/backend
- control loop rate/deadband/move-time defaults
- limitations and expected unsupported paths
- any new files introduced in platform/drivers/hal/app
- BLE mode commands, packet sizes, and units
- Cartesian kinematics constants, mapping, and failure behavior
