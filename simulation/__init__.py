"""
Robotic Arm Simulation Package

This package provides a modular framework for simulating and visualizing
robotic arms. It includes:

- models: Robot arm definitions and kinematics
- config: Configuration classes and parameters
- core: Core utilities including constraints
- visualization: 3D viewers and animation
- kinematics: Forward and inverse kinematics (future)
- scripts: Runnable demos and utilities

Quick Start:
    from simulation.models.three_link_arm import ThreeLinkArm
    from simulation.visualization.view_robot import ArmViewer
    
    arm = ThreeLinkArm()
    viewer = ArmViewer(arm)
    viewer.show()

Author: Senior Design Team
Date: January 2026
"""

__version__ = "0.1.0"
__author__ = "Senior Design Team"
