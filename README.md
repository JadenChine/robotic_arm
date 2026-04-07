# SD403_Sp26_05 Robotic Arm

This repository houses the code and files related to SD403_Sp26_05. This design project sets up the controls and kinematics of a 6 DOF robotic arm.

## Repository Layout

- `src/`: Embedded and hardware-facing code.
- `sim/`: Python simulation, visualization, and simulation tests.
- `Design_Doc.md`: Design and task breakdown.

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
