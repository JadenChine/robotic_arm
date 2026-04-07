# Senior_Design_Robotic_Arm

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
