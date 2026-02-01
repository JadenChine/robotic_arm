# 3-Link Robotic Arm Simulation

A modular Python simulation framework for a 3-DOF robotic arm built using the Robotics Toolbox for Python.

## Overview

This simulation models a 3-link serial manipulator with:
- **Base joint (yaw)**: 360° rotation in the XY plane
- **Shoulder joint (pitch)**: 0° to 180° elevation (constrained to upper hemisphere)
- **Elbow joint (pitch)**: ±90° articulation

### Coordinate System
- **Origin**: Robot base at (0, 0, 0)
- **XY plane**: Ground/base plane
- **Z-axis**: Positive upward
- **Total reach**: 9 units (3 units per link)

## Project Structure

```
simulation/
├── __init__.py              # Package initialization
├── models/
│   ├── __init__.py
│   ├── 3_link_arm.py        # Main robot model (primary file)
│   └── three_link_arm.py    # Python-friendly import alias
├── config/
│   ├── __init__.py
│   └── robot_params.py      # Robot configuration classes
├── core/
│   ├── __init__.py
│   └── constraints.py       # Constraint system (joint limits, collision)
├── visualization/
│   ├── __init__.py
│   └── view_robot.py        # Interactive 3D matplotlib viewer
├── kinematics/
│   ├── __init__.py
│   ├── fk_tests.py          # Forward kinematics tests (placeholder)
│   └── ik_tests.py          # Inverse kinematics tests (placeholder)
├── scripts/
│   ├── __init__.py
│   ├── manual_control.py    # Mode 1: Interactive joint control
│   └── automated_demo.py    # Mode 2: Automated movement sequences
└── tests/
    ├── __init__.py
    └── test_arm.py          # Unit tests
```

## Quick Start

### Installation

Ensure you have the required dependencies installed:

```bash
cd Senior_Design_Robotic_Arm
pipenv install
```

### Running the Viewer

**Mode 1: Manual Control**
```bash
pipenv run python simulation/scripts/manual_control.py
```

Use the sliders to control each joint:
- `base_yaw`: Base rotation (-180° to 180°)
- `shoulder_pitch`: Shoulder elevation (0° to 180°)
- `elbow_pitch`: Elbow bend (-90° to 90°)

**Mode 2: Automated Demo**
```bash
pipenv run python simulation/scripts/automated_demo.py
```

Choose from multiple demo modes:
- `basic`: Range of motion demonstration
- `sweep`: Workspace coverage sweep
- `reach`: Maximum reach test
- `pick_place`: Simulated pick-and-place

### Using the Model Programmatically

```python
from simulation.models.three_link_arm import ThreeLinkArm
import numpy as np

# Create arm
arm = ThreeLinkArm()
print(arm.summary())

# Set joint angles (radians)
arm.set_joint_angles([0, np.pi/2, 0])  # Home position

# Get end effector position
pos = arm.get_end_effector_position()
print(f"End effector: {pos}")

# Check constraints
state = arm.get_state()
if not state.is_valid:
    print("Constraint violations:", state.constraint_violations)

# Move single joint
arm.move_joint(0, np.pi/4)  # Rotate base 45°
```

## Joint Specifications

| Joint | Name | Type | Range | Home |
|-------|------|------|-------|------|
| 0 | base_yaw | Revolute (Z) | -180° to 180° | 0° |
| 1 | shoulder_pitch | Revolute (Y) | 0° to 180° | 90° |
| 2 | elbow_pitch | Revolute (Y) | -90° to 90° | 0° |

## Constraints

### Hard Stops (Joint Limits)
Each joint has enforced limits. The `set_joint_angles()` method automatically clamps values.

### Ground Collision
The arm is prevented from going below the XY plane (Z < 0). The constraint system detects when any joint or the end effector would penetrate the ground plane.

### Workspace
The maximum workspace is a hemisphere of radius 9 units centered at the origin.

## Extending the Model

### Adding More Joints

Use the `create_n_link_config()` factory in `config/robot_params.py`:

```python
from simulation.config.robot_params import create_n_link_config

config = create_n_link_config(n_links=5, link_length=2.0)
```

### Custom Configurations

Create a custom `RobotConfig`:

```python
from simulation.config.robot_params import (
    RobotConfig, JointConfig, LinkConfig, JointType
)
import numpy as np

config = RobotConfig(
    name="Custom Arm",
    joints=[
        JointConfig("joint1", JointType.REVOLUTE, 'z', -np.pi, np.pi),
        # ... more joints
    ],
    links=[
        LinkConfig("link1", 5.0),
        # ... more links
    ]
)
```

### Adding New Constraints

Extend the `Constraint` base class in `core/constraints.py`:

```python
from simulation.core.constraints import Constraint, ConstraintType

class MyConstraint(Constraint):
    @property
    def constraint_type(self):
        return ConstraintType.CUSTOM
    
    def check(self, joint_angles, joint_positions=None):
        # Return list of violations
        pass
    
    def correct(self, joint_angles, joint_positions=None):
        # Return corrected angles
        pass
```

## Running Tests

```bash
pipenv run python simulation/tests/test_arm.py
```

## Dependencies

- `numpy` < 2.0
- `matplotlib`
- `roboticstoolbox-python`
- `spatialmath-python` (installed with roboticstoolbox)

## Future Development

- [ ] Inverse kinematics solvers
- [ ] Trajectory planning
- [ ] Dynamics simulation
- [ ] More visualization options (Swift viewer)
- [ ] Self-collision detection
- [ ] Integration with hardware control

## Authors

Senior Design Team - January 2026
