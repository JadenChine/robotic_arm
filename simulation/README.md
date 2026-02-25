
# 3-Link Robotic Arm Simulation
A modular Python simulation framework for a 4-DOF robotic arm built using the Robotics Toolbox for Python.

## Table of Contents
- [Overview](#overview)
- [Detailed Arm Kinematics & Physical Description](#detailed-arm-kinematics--physical-description)
  - [Physical Configuration](#physical-configuration)
  - [Link Specifications](#link-specifications)
  - [Joint Specifications (Detailed)](#joint-specifications-detailed)
  - [Motion Conventions (Industry Standard)](#motion-conventions-industry-standard)
  - [Workspace Description](#workspace-description)
  - [Visualization Reference](#visualization-reference)
- [Constraints](#constraints)
  - [Hard Stops (Joint Limits)](#hard-stops-joint-limits)
  - [Ground Collision](#ground-collision)
  - [Workspace](#workspace)
- [Project Structure](#project-structure)
- [Quick Start](#quick-start)
  - [Running the Viewer](#running-the-viewer)
  - [Running Tests](#running-tests)
  - [Using the Model Programmatically](#using-the-model-programmatically)
- [Extending the Model](#extending-the-model)
  - [Adding More Joints](#adding-more-joints)
  - [Custom Configurations](#custom-configurations)
  - [Adding New Constraints](#adding-new-constraints)


## Overview
This simulation models a 3-link serial manipulator with 4 degrees of freedom:

**Coordinate System**
- **Origin**: Robot base at (0, 0, 0)
- **XY plane**: Ground/base plane (floor plane, the robot is built vertically on top of the ground)
- **Z-axis**: Vertical plane (3rd dimension extending upward from the floor)

**Joints**
- **Joint 1 - Base Yaw (Shoulder Part 1)**: ±180° rotation about the Z axis
- **Joint 2 - Shoulder Pitch (Shoulder Part 2)**: ±90° pitch (0° is vertical/up, ±90° is horizontal along the ±X direction)
- **Joint 3 - Elbow 1**: ±90° articulation (first elbow joint)
- **Joint 4 - Elbow 2**: ±90° articulation (second elbow joint)

**Note**: The shoulder is a compound 2-in-1 joint (joints 1 & 2 at the same physical location at the base).


## Detailed Arm Kinematics & Physical Description

### Physical Configuration
The arm consists of **3 links** and **4 revolute joints** arranged as a serial chain:

```
Base (Shoulder) → Link 1 (Upper Arm) → Link 2 (Forearm) → Link 3 (Hand/Tool)
     ↓                    ↓                    ↓                    ↓
  Joint 1-2           Joint 3              Joint 4           End Effector
(Compound Shoulder)  (Elbow 1)            (Elbow 2)
```

### Link Specifications
| Link | Name | Length | Connects | Description |
|------|------|--------|----------|-------------|
| Link 1 | Upper Arm | 3.0 units | Shoulder → Elbow 1 | First segment extending from compound shoulder joint |
| Link 2 | Forearm | 3.0 units | Elbow 1 → Elbow 2 | Middle segment between two elbow joints |
| Link 3 | Hand/Tool | 3.0 units | Elbow 2 → End Effector | Final segment terminating at end effector |

**Total Serial Length**: 9.0 units (fully extended reach)

### Joint Specifications (Detailed)

#### Joint 1: Base Yaw (Shoulder - Part 1)
- **Type**: Revolute
- **Axis of Rotation**: Z-axis (vertical)
- **Location**: Base origin (0, 0, 0)
- **Range**: -180° to +180° (-π to +π radians)
- **Home Position**: 0° (arm oriented toward +X axis)
- **Motion Description**: 
  - **Positive rotation**: Counter-clockwise when viewed from above (right-hand rule about +Z)
  - **Industry terms**: Azimuth, horizontal rotation, base swivel
  - **Function**: Positions the entire arm in the XY plane
- **Physical behavior**: Rotates the entire arm assembly (Link 1, 2, 3) around the vertical axis

#### Joint 2: Shoulder Pitch (Shoulder - Part 2)
- **Type**: Revolute
- **Axis of Rotation**: Y-axis (after base yaw rotation)
- **Location**: Base origin (0, 0, 0) - **same location as Joint 1**
- **Range**: -90° to +90° (-π/2 to π/2 radians)
- **Home Position**: 0° ( radians) - arm vertical
- **Motion Description**:
  - **0° position**: Link 1 points straight up (+Z direction, vertical)
  - **90° position**: Link 1 points forward horizontally (+X direction assuming yaw is 0°)
  - **-90° position**: Link 1 points backward horizonatlly (-X direction assuming yaw is 0°)
  - **Positive rotation**: From vertical (+Z) toward forward horizontal in +X direction
  - **Negative rotation**: From vertical (+Z) toward backward horizontal in -X direction
  - **Constraint**: Upper hemisphere operation (-90° to 90° where 0° is vertical, cannot go below ground)
- **Physical behavior**: Controls the elevation angle of the entire arm chain

#### Joint 3: Elbow 1 Pitch (First Elbow)
- **Type**: Revolute
- **Axis of Rotation**: Y-axis (parallel to shoulder pitch axis)
- **Location**: End of Link 1 (3 units from base along Link 1's direction)
- **Range**: -90° to +90° (-π/2 to +π/2 radians)
- **Home Position**: 0° (Link 2 extends straight along Link 1 direction)
- **Motion Description**:
  - **0° position**: Link 2 fully extended in line with Link 1 (no bend)
  - **Positive rotation**: Link 2 bends forward (toward +X when shoulder is at 0°)
  - **Negative rotation**: Link 2 bends backward (toward -X when shoulder is at 0°)
  - **Industry terms**: Elbow flexion (positive), elbow extension (negative from bent)
- **Physical behavior**: First articulation point allowing arm to fold/bend

#### Joint 4: Elbow 2 Pitch (Second Elbow)
- **Type**: Revolute
- **Axis of Rotation**: Y-axis (parallel to other pitch axes, after base yaw rotation)
- **Location**: End of Link 2 (6 units from base through Link 1 and Link 2)
- **Range**: -90° to +90° (-π/2 to +π/2 radians)
- **Home Position**: 0° (Link 3 extends straight along Link 2 direction)
- **Motion Description**:
  - **0° position**: Link 3 fully extended in line with Link 2 (no bend)
  - **Positive rotation**: Link 3 bends forward (continuing the bend from Elbow 1)
  - **Negative rotation**: Link 3 bends backward (can straighten or counter-bend)
  - **Industry terms**: Secondary elbow flexion
  - **Behavior**: Cumulative effect with Elbow 1; both positive angles increase forward bend
- **Physical behavior**: Second articulation allowing fine positioning and complex arm shapes


### Motion Conventions (Industry Standard)
**Rotation Directions** (Right-Hand Rule):
- **Yaw** (Joint 1): Rotation about Z-axis
  - Thumb points up (+Z), fingers curl in positive rotation direction (CCW from above)
- **Pitch** (Joints 2, 3, 4): Rotation about Y-axis
  - Thumb points left (+Y direction), fingers curl from up (+Z) toward forward (+X)


### Workspace Description
**Reachable Workspace**:
- **Shape**: Hemisphere (upper half) in the positive Z plane with a 9 unit radius
- **Maximum radius**: 9 units (all links fully extended)
- **Minimum radius**: ~3 units (maximum fold back, limited by joint ranges)
- **Volume**: Constrained by ground plane (Z ≥ 0 enforced)


### Visualization Reference
**Key Configurations**:
1. **Home Position** [0°, 0°, 0°, 0°]: Arm vertical, fully extended vertical 
2. **Vertical Up** [any°, 0°, 0°, 0°]: Arm pointing straight up
3. **Maximum Reach** [any, 90°, 0°, 0°]: 9 units horizontal (points to +X if yaw is 0°)
4. **Folded** [0°, 0°, 90°, 90°]: Compact, fully bent forward, end effector at (3,0,0)


## Constraints

### Hard Stops (Joint Limits)
Each joint has enforced limits. The `set_joint_angles()` method automatically clamps values.

### Ground Collision
The arm is prevented from going below the XY plane (Z < 0). The constraint system detects when any joint or the end effector would penetrate the ground plane.

### Workspace
The maximum workspace is a hemisphere of radius 9 units centered at the origin.


## Project Structure
```
simulation/
├── __init__.py              # Package initialization
├── models/
│   ├── __init__.py
│   └── three_link_arm.py    # Main robot model (primary file)
├── config/
│   ├── __init__.py
│   └── robot_params.py      # Base robot configuration classes
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

### Running the Viewer
**Mode 1: Manual Control**
```bash
pipenv run python simulation/scripts/manual_control.py
```

Use the sliders to control each joint:
- `base_yaw`: Base rotation (-180° to 180°) - Shoulder Part 1
- `shoulder_pitch`: Shoulder elevation (-90° to 90°) - Shoulder Part 2
- `elbow_1_pitch`: First elbow bend (-90° to 90°)
- `elbow_2_pitch`: Second elbow bend (-90° to 90°)

**Mode 2: Automated Demo**
```bash
pipenv run python simulation/scripts/automated_demo.py
```

Choose from multiple demo modes:
- `basic`: Range of motion demonstration
- `sweep`: Workspace coverage sweep
- `reach`: Maximum reach test
- `pick_place`: Simulated pick-and-place

### Running Tests
```bash
pipenv run python simulation/tests/test_arm.py
```

### Using the Model Programmatically
```python
from simulation.models.three_link_arm import ThreeLinkArm
import numpy as np

# Create arm
arm = ThreeLinkArm()
print(arm.summary())

# Set joint angles (radians) - [base_yaw, shoulder_pitch, elbow_1, elbow_2]
arm.set_joint_angles([0, np.pi/2, 0, 0])  # Home position

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
