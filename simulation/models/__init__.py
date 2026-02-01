"""
Robot Models Module

Contains robotic arm model definitions including:
- ThreeLinkArm: 4-DOF arm with 3 links (compound shoulder + 2 elbows)

Usage:
    from simulation.models.three_link_arm import ThreeLinkArm
    
    arm = ThreeLinkArm()
    arm.go_home()
    pos = arm.get_end_effector_position()
"""

from .three_link_arm import ThreeLinkArm, create_arm, ArmState

__all__ = ['ThreeLinkArm', 'create_arm', 'ArmState']
