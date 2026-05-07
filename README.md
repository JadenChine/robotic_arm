# SD403_Sp26_05 Robotic Arm

This repository houses the code and files related to SD403_Sp26_05. This design project sets up the controls and kinematics of a 6 DOF robotic arm.

## Repository Layout

- `src/`: Embedded and hardware-facing code.
- `sim/`: Python simulation, visualization, and simulation tests.

## Simulation Quick Start

From the repository root:

```bash
pipenv run python sim/scripts/manual_control.py
pipenv run python sim/scripts/automated_demo.py
pipenv run python sim/scripts/test_arm.py
```

## Pico Project Quick Start

The Pico firmware lives in `src/arm/`.

From that directory:

```bash
cmake -S . -B build
cmake --build build
```

To flash the generated UF2 after building:

```bash
./scripts/build_and_flash.sh
./scripts/build_and_flash.sh --connect
```

To open the USB serial monitor directly:

```bash
./scripts/serial_connect.sh
```

The `build/` directory will typically contain the CMake cache and generated build system files, intermediate object files, dependency metadata, the linked ELF image, and the exported outputs such as `.uf2`, `.bin`, `.hex`, `.map`, and `.dis` files.

## Initial Servo Control Bootstrap

The arm firmware includes a backend-agnostic servo-bus HAL and an application-layer joint-target control loop under `src/arm/`.

### Architecture

- `src/arm/src/platform/`: Pico-specific UART transport and hardware defaults
- `src/arm/src/drivers/`: internal HiWonder packet protocol driver
- `src/arm/src/hal/`: C-facing hardware abstraction layer for the rest of the arm firmware
- `src/arm/main.c`: 6-joint periodic target controller + heartbeat LED; BLE runs on core1 on Pico W

For detailed firmware behavior and the BLE protocol, see `src/arm/README.md` and `src/arm/docs/BLE_README.md`.

The reference code in `src/HiBusServo/` is used only as source material during the port. The long-term arm firmware should depend on the `src/arm/` implementation instead.

### Default hardware assumptions

- Servo bus backend: controller mode
- Servo bus UART index: `0`
- Servo bus baud rate: `9600`
- Servo bus TX pin: `GPIO 0` (GP0)
- Servo bus RX pin: `GPIO 1` (GP1)
- Default servo ID (for low-level tools): `1`
- Heartbeat LED pin: `GPIO 16`

These defaults are centralized in `src/arm/src/platform/servo_platform_config.h`.

### Environment requirements

To build the Pico firmware on a new machine, you need a working Pico toolchain in addition to the SDK submodule:

- `arm-none-eabi-gcc`
- a working CMake generator such as `ninja` or `make`

The USB serial helper script also expects Python with `pyserial` available. It will use `pipenv run python` when `pipenv` exists, otherwise it falls back to `python3`.

### Current status

This is the bootstrap for the low-level servo HAL only. It is intended to prove transport, packet handling, basic motion, and telemetry before adding joint mapping, calibration, or higher-level arm motion logic.
