"""
Robot Parameters Configuration Module

This module defines the data classes and default configurations for robotic arms.
All measurements are in standard SI units (meters, radians) unless otherwise specified.

The configuration is designed to be:
- Easily extensible for additional joints
- Serializable for saving/loading configurations
- Validated at construction time

Author: Jaden Chine
Date: January 31st 2026
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum
import numpy as np


class JointType(Enum):
    """Enumeration of supported joint types."""
    REVOLUTE = "revolute"      # Rotational joint
    PRISMATIC = "prismatic"    # Linear sliding joint


@dataclass
class JointConfig:
    """
    Configuration for a single robot joint.
    
    Attributes:
        name: Human-readable identifier for the joint
        joint_type: Type of joint (revolute or prismatic)
        axis: Axis of rotation/translation ('x', 'y', 'z')
        limit_lower: Lower limit in radians (revolute) or meters (prismatic)
        limit_upper: Upper limit in radians (revolute) or meters (prismatic)
        home_position: Default/home position of the joint
        max_velocity: Maximum angular/linear velocity (rad/s or m/s)
        description: Optional description of joint function
    """
    name: str
    joint_type: JointType
    axis: str
    limit_lower: float
    limit_upper: float
    home_position: float = 0.0
    max_velocity: float = np.pi  # Default: 180 deg/s
    description: str = ""
    
    def __post_init__(self):
        """Validate joint configuration after initialization."""
        if self.axis not in ('x', 'y', 'z'):
            raise ValueError(f"Axis must be 'x', 'y', or 'z', got '{self.axis}'")
        if self.limit_lower >= self.limit_upper:
            raise ValueError(f"Lower limit must be less than upper limit")
        if not self.limit_lower <= self.home_position <= self.limit_upper:
            raise ValueError(f"Home position must be within joint limits")
    
    @property
    def range(self) -> float:
        """Total range of motion for this joint."""
        return self.limit_upper - self.limit_lower
    
    def clamp(self, value: float) -> float:
        """Clamp a value to within joint limits."""
        return np.clip(value, self.limit_lower, self.limit_upper)
    
    def is_within_limits(self, value: float) -> bool:
        """Check if a value is within joint limits."""
        return self.limit_lower <= value <= self.limit_upper


@dataclass
class LinkConfig:
    """
    Configuration for a single robot link.
    
    Attributes:
        name: Human-readable identifier for the link
        length: Length of the link in meters (or units)
        offset_d: DH parameter d - offset along previous z-axis
        offset_a: DH parameter a - offset along rotated x-axis  
        twist_alpha: DH parameter alpha - rotation about x-axis
        mass: Mass of the link (for dynamics, optional)
        description: Optional description of link function
    """
    name: str
    length: float
    offset_d: float = 0.0
    offset_a: float = 0.0
    twist_alpha: float = 0.0
    mass: float = 1.0
    description: str = ""
    
    def __post_init__(self):
        """Validate link configuration after initialization."""
        if self.length < 0:
            raise ValueError(f"Link length must be non-negative")


@dataclass
class RobotConfig:
    """
    Complete configuration for a serial-link robotic arm.
    
    This class holds all parameters needed to construct and simulate
    a robotic arm, including joints, links, and global constraints.
    
    Attributes:
        name: Robot identifier
        joints: List of joint configurations
        links: List of link configurations
        base_position: Position of robot base in world frame (x, y, z)
        ground_plane_z: Z-coordinate of ground plane (constraint boundary)
        enable_ground_collision: Whether to enforce ground collision avoidance
    """
    name: str
    joints: List[JointConfig]
    links: List[LinkConfig]
    base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ground_plane_z: float = 0.0
    enable_ground_collision: bool = True
    
    def __post_init__(self):
        """Validate robot configuration after initialization."""
        # Note: joints and links don't need to be equal
        # A 3-link arm can have 4 joints (e.g., 2 shoulder + 2 elbow)
        if len(self.joints) < 1 or len(self.links) < 1:
            raise ValueError(
                f"Must have at least 1 joint and 1 link"
            )
    
    @property
    def num_joints(self) -> int:
        """Number of joints/DOF in the robot."""
        return len(self.joints)
    
    @property
    def total_reach(self) -> float:
        """Maximum reach of the robot (sum of all link lengths)."""
        return sum(link.length for link in self.links)
    
    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get arrays of joint limits.
        
        Returns:
            Tuple of (lower_limits, upper_limits) as numpy arrays
        """
        lower = np.array([j.limit_lower for j in self.joints])
        upper = np.array([j.limit_upper for j in self.joints])
        return lower, upper
    
    def get_home_position(self) -> np.ndarray:
        """Get the home position configuration."""
        return np.array([j.home_position for j in self.joints])
    
    def clamp_joints(self, q: np.ndarray) -> np.ndarray:
        """
        Clamp joint values to within their limits.
        
        Args:
            q: Joint configuration to clamp
            
        Returns:
            Clamped joint configuration
        """
        return np.array([
            joint.clamp(val) for joint, val in zip(self.joints, q)
        ])


# =============================================================================
# DEFAULT CONFIGURATIONS
# =============================================================================

# 3-Link Arm Configuration
# This configuration represents a 4-DOF arm with 3 links:
# - Joint 1 (Base Yaw): Rotation about Z-axis, 360° range - part of shoulder
# - Joint 2 (Shoulder Pitch): Rotation about Y-axis (pitch), -90° to 90° - part of shoulder
#       0° = straight up, negative = forward, positive = backward
# - Joint 3 (Elbow 1): First elbow, rotation about Y-axis, ±90° range
# - Joint 4 (Elbow 2): Second elbow, rotation about Y-axis, ±90° range
#
# The shoulder is a compound joint (joints 1 & 2 together)
# Links: 3 units each, total reach 9 units

DEFAULT_3_LINK_CONFIG = RobotConfig(
    name="3-Link Arm",
    joints=[
        JointConfig(
            name="base_yaw",
            joint_type=JointType.REVOLUTE,
            axis='z',
            limit_lower=-np.pi,           # -180 degrees
            limit_upper=np.pi,            # +180 degrees
            home_position=0.0,
            max_velocity=np.pi,           # 180 deg/s
            description="Shoulder joint part 1: Base rotation about Z-axis (XY plane rotation)"
        ),
        JointConfig(
            name="shoulder_pitch",
            joint_type=JointType.REVOLUTE,
            axis='y',
            limit_lower=-np.pi/2,         # -90 degrees (horizontal forward)
            limit_upper=np.pi/2,          # +90 degrees (horizontal backward)
            home_position=0.0,            # Start pointing straight up
            max_velocity=np.pi,
            description="Shoulder joint part 2: Pitch - 0° is straight up, ±90° is horizontal"
        ),
        JointConfig(
            name="elbow_1_pitch",
            joint_type=JointType.REVOLUTE,
            axis='y',
            limit_lower=-np.pi/2,         # -90 degrees
            limit_upper=np.pi/2,          # +90 degrees
            home_position=0.0,
            max_velocity=np.pi,
            description="First elbow joint - ±90° bend forward/backward"
        ),
        JointConfig(
            name="elbow_2_pitch",
            joint_type=JointType.REVOLUTE,
            axis='y',
            limit_lower=-np.pi/2,         # -90 degrees
            limit_upper=np.pi/2,          # +90 degrees
            home_position=0.0,
            max_velocity=np.pi,
            description="Second elbow joint - ±90° bend forward/backward"
        ),
    ],
    links=[
        LinkConfig(
            name="link_1",
            length=3.0,
            description="Upper arm - connects shoulder to first elbow"
        ),
        LinkConfig(
            name="link_2", 
            length=3.0,
            description="Forearm - connects first elbow to second elbow"
        ),
        LinkConfig(
            name="link_3",
            length=3.0,
            description="Hand/end effector link - connects second elbow to end effector"
        ),
    ],
    base_position=(0.0, 0.0, 0.0),
    ground_plane_z=0.0,
    enable_ground_collision=True
)


def create_n_link_config(
    n_links: int,
    link_length: float = 3.0,
    name: str = "N-Link Arm"
) -> RobotConfig:
    """
    Factory function to create an n-link arm configuration.
    
    Creates a robot with:
    - First joint: Base yaw (Z-axis rotation, ±180°)
    - Second joint: Shoulder pitch (Y-axis, 0° to 180° for upper hemisphere)
    - Remaining joints: Elbow pitch (Y-axis, ±90°)
    
    Args:
        n_links: Number of links (and joints)
        link_length: Length of each link
        name: Name for the robot configuration
        
    Returns:
        RobotConfig for the specified arm
    """
    if n_links < 1:
        raise ValueError("Must have at least 1 link")
    
    joints = []
    links = []
    
    for i in range(n_links):
        if i == 0:
            # Base yaw joint
            joints.append(JointConfig(
                name=f"joint_{i+1}_base_yaw",
                joint_type=JointType.REVOLUTE,
                axis='z',
                limit_lower=-np.pi,
                limit_upper=np.pi,
                home_position=0.0,
                description=f"Joint {i+1}: Base rotation"
            ))
        elif i == 1:
            # Shoulder pitch - constrained to upper hemisphere
            joints.append(JointConfig(
                name=f"joint_{i+1}_shoulder_pitch",
                joint_type=JointType.REVOLUTE,
                axis='y',
                limit_lower=0.0,
                limit_upper=np.pi,
                home_position=np.pi/2,
                description=f"Joint {i+1}: Shoulder pitch (upper hemisphere)"
            ))
        else:
            # Elbow joints
            joints.append(JointConfig(
                name=f"joint_{i+1}_elbow_pitch",
                joint_type=JointType.REVOLUTE,
                axis='y',
                limit_lower=-np.pi/2,
                limit_upper=np.pi/2,
                home_position=0.0,
                description=f"Joint {i+1}: Elbow pitch (±90°)"
            ))
        
        links.append(LinkConfig(
            name=f"link_{i+1}",
            length=link_length,
            description=f"Link {i+1}"
        ))
    
    return RobotConfig(
        name=name,
        joints=joints,
        links=links,
        enable_ground_collision=True
    )
