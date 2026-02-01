"""
Configuration module for robotic arm parameters.

This module contains all configurable parameters for the robot including:
- Link lengths and dimensions
- Joint limits and types
- Physical constraints
"""

from .robot_params import (
    RobotConfig,
    JointConfig,
    LinkConfig,
    DEFAULT_3_LINK_CONFIG,
)

__all__ = [
    'RobotConfig',
    'JointConfig', 
    'LinkConfig',
    'DEFAULT_3_LINK_CONFIG',
]
