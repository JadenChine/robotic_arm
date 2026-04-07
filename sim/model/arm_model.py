"""
3-Link Robotic Arm Model

This module defines the core robotic arm model using the Robotics Toolbox for Python.
The arm is configured as a 4-DOF serial manipulator with 3 links:
    - Joint 1 (Base Yaw): Z-axis rotation for XY-plane positioning - part of shoulder
    - Joint 2 (Shoulder Pitch): Y-axis rotation for elevation (upper hemisphere only) - part of shoulder
    - Joint 3 (Elbow 1): Y-axis rotation, first elbow joint (±90°)
    - Joint 4 (Elbow 2): Y-axis rotation, second elbow joint (±90°)

The shoulder is a compound 2-in-1 joint (joints 1 & 2 together at the base).

The coordinate system follows:
    - XY plane: Ground/base plane
    - Z-axis: Positive upward
    - Origin: Robot base at (0, 0, 0)

All lengths are expressed in millimeters. The default links are 101, 95, and
168 mm, giving a total reach of 364 mm when fully extended.

Usage:
    from sim.model.arm_model import ThreeLinkArm
    
    arm = ThreeLinkArm()
    arm.set_joint_angles([0, np.pi/2, 0, 0])  # Home position
    end_effector_pos = arm.forward_kinematics()

Author: Senior Design Team
Date: January 2026
"""

import numpy as np
from typing import Optional, Tuple, List, Union
from dataclasses import dataclass

# Robotics Toolbox imports
import roboticstoolbox as rtb
from roboticstoolbox import ET, Robot
from spatialmath import SE3

from robot_params import RobotConfig, DEFAULT_3_LINK_CONFIG


@dataclass
class ArmState:
    """
    Represents the complete state of the robotic arm.
    
    Attributes:
        joint_angles: Current joint angles in radians
        end_effector_pose: SE3 transformation of end effector
        joint_positions: List of (x, y, z) positions for each joint
        is_valid: Whether the current state satisfies all constraints
        constraint_violations: List of any constraint violations
    """
    joint_angles: np.ndarray
    end_effector_pose: Optional[SE3] = None
    joint_positions: Optional[List[Tuple[float, float, float]]] = None
    is_valid: bool = True
    constraint_violations: List[str] = None
    
    def __post_init__(self):
        if self.constraint_violations is None:
            self.constraint_violations = []


class ThreeLinkArm:
    """
    3-Link Robotic Arm Model (4 DOF)
    
    A configurable 4-DOF robotic arm with 3 links, built using the Robotics Toolbox.
    Supports forward kinematics, joint limit enforcement, and ground
    collision avoidance.
    
    Joint configuration:
    - Joints 1 & 2: Compound shoulder (yaw + pitch)
    - Joint 3: First elbow
    - Joint 4: Second elbow
    
    The arm uses Elementary Transform Sequence (ETS) notation for
    maximum flexibility in defining the kinematic chain.
    
    Attributes:
        config: RobotConfig containing joint/link parameters
        robot: RTB Robot object for kinematics calculations
        current_angles: Current joint configuration
    """
    
    def __init__(self, config: Optional[RobotConfig] = None):
        """
        Initialize the 3-link arm.
        
        Args:
            config: Robot configuration. Uses DEFAULT_3_LINK_CONFIG if None.
        """
        self.config = config or DEFAULT_3_LINK_CONFIG
        self._validate_config()
        
        # Build the robot model
        self.robot = self._build_robot()
        
        # Initialize to home position
        self._current_angles = self.config.get_home_position()
        
    def _validate_config(self):
        """Validate that the configuration is suitable for this arm type."""
        if self.config.num_joints != 4:
            raise ValueError(
                f"ThreeLinkArm requires exactly 4 joints (2 shoulder + 2 elbow), "
                f"got {self.config.num_joints}"
            )
    
    def _build_robot(self) -> Robot:
        """
        Build the RTB Robot model from configuration.
        
        The robot is constructed using Elementary Transform Sequences (ETS)
        which allows for flexible specification of the kinematic chain.
        
        Returns:
            Configured RTB Robot object
        """
        # Get link lengths from config
        L1 = self.config.links[0].length  # First link (upper arm)
        L2 = self.config.links[1].length  # Second link (forearm)
        L3 = self.config.links[2].length  # Third link (hand)
        
        # Get joint limits
        j1 = self.config.joints[0]  # Base yaw (shoulder part 1)
        j2 = self.config.joints[1]  # Shoulder pitch (shoulder part 2)
        j3 = self.config.joints[2]  # Elbow 1
        j4 = self.config.joints[3]  # Elbow 2
        
        # Build the Elementary Transform Sequence
        # 
        # ETS chain (all transforms are relative to the previous local frame):
        # - Base at origin (0, 0, 0)
        # - Joint 1 (Base Yaw): Rotates entire arm in XY plane about Z-axis
        # - Constant -90° Y-rotation: Aligns local X-axis with vertical direction
        #   (required so ET.tx translations extend upward when shoulder_pitch = 0°)
        # - Joint 2 (Shoulder Pitch): 0° = vertical, +90° = horizontal forward
        # - All links extend along local X-axis via ET.tx()
        # - Elbows bend in pitch plane (positive = forward flexion)
        
        ets = (
            # Joint 1: Base yaw rotation (rotates in ground plane)
            ET.Rz(qlim=[j1.limit_lower, j1.limit_upper]) *
            
            # Constant coordinate frame alignment: rotates local X to point vertically
            # This allows ET.tx() translations to extend upward when shoulder is at 0°
            ET.Ry(-np.pi/2) *
            
            # Joint 2: Shoulder pitch (0° = vertical up, +90° = horizontal forward)
            ET.Ry(qlim=[j2.limit_lower, j2.limit_upper]) *
            
            # Link 1: Upper arm extends along local X-axis
            ET.tx(L1) *
            
            # Joint 3: First elbow pitch (positive = forward bend)
            ET.Ry(qlim=[j3.limit_lower, j3.limit_upper]) *
            
            # Link 2: Forearm extends along local X-axis
            ET.tx(L2) *
            
            # Joint 4: Second elbow pitch (positive = forward bend)
            ET.Ry(qlim=[j4.limit_lower, j4.limit_upper]) *
            
            # Link 3: Hand/end effector extends along local X-axis
            ET.tx(L3)
        )
        
        # Create the robot
        robot = rtb.Robot(
            ets,
            name=self.config.name,
        )
        
        return robot
    
    @property
    def current_angles(self) -> np.ndarray:
        """Get current joint angles."""
        return self._current_angles.copy()
    
    @current_angles.setter
    def current_angles(self, angles: np.ndarray):
        """Set joint angles with limit enforcement."""
        self._current_angles = self.clamp_to_limits(np.asarray(angles))
    
    @property
    def num_joints(self) -> int:
        """Number of joints in the arm."""
        return self.config.num_joints
    
    @property
    def total_reach(self) -> float:
        """Maximum reach of the arm in millimeters."""
        return self.config.total_reach
    
    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get joint limits.
        
        Returns:
            Tuple of (lower_limits, upper_limits) arrays
        """
        return self.config.get_joint_limits()
    
    def clamp_to_limits(self, angles: np.ndarray) -> np.ndarray:
        """
        Clamp joint angles to within their limits.
        
        Args:
            angles: Joint angles to clamp
            
        Returns:
            Clamped joint angles
        """
        return self.config.clamp_joints(angles)
    
    def is_within_limits(self, angles: np.ndarray) -> bool:
        """
        Check if joint angles are within limits.
        
        Args:
            angles: Joint angles to check
            
        Returns:
            True if all angles are within limits
        """
        lower, upper = self.get_joint_limits()
        return np.all(angles >= lower) and np.all(angles <= upper)
    
    def forward_kinematics(
        self, 
        angles: Optional[np.ndarray] = None
    ) -> SE3:
        """
        Compute forward kinematics.
        
        Args:
            angles: Joint angles. Uses current angles if None.
            
        Returns:
            SE3 transformation representing end effector pose
        """
        if angles is None:
            angles = self._current_angles
        else:
            angles = np.asarray(angles)
            
        return self.robot.fkine(angles)
    
    def get_end_effector_position(
        self, 
        angles: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Get the (x, y, z) position of the end effector.
        
        Args:
            angles: Joint angles. Uses current angles if None.
            
        Returns:
            3D position array [x, y, z]
        """
        pose = self.forward_kinematics(angles)
        return pose.t
    
    def get_all_joint_positions(
        self, 
        angles: Optional[np.ndarray] = None
    ) -> List[np.ndarray]:
        """
        Get the positions of all joints and end effector.
        
        Args:
            angles: Joint angles. Uses current angles if None.
            
        Returns:
            List of 3D positions: [base, end_of_link1, end_of_link2, end_effector]
        """
        if angles is None:
            angles = self._current_angles
        else:
            angles = np.asarray(angles)
            
        positions = [np.array([0.0, 0.0, 0.0])]  # Base position (shoulder location)
        
        # Get link lengths
        L1 = self.config.links[0].length
        L2 = self.config.links[1].length
        L3 = self.config.links[2].length
        
        # Joint angles
        theta1 = angles[0]  # Base yaw (shoulder part 1)
        theta2 = angles[1]  # Shoulder pitch (shoulder part 2): 0° = up, +90° = forward
        theta3 = angles[2]  # Elbow 1 pitch: positive = bend forward
        theta4 = angles[3]  # Elbow 2 pitch: positive = bend forward
        
        # Convert shoulder pitch to the internal angle for FK
        # theta2 = 0 means straight up, positive angles bend forward toward +X
        # We use π/2 - theta2 so that:
        #   - theta2 = 0: effective_pitch = π/2, sin(π/2) = 1 (pointing up)
        #   - theta2 = +90°: effective_pitch = 0, cos(0) = 1 (pointing forward +X)
        effective_pitch = np.pi/2 - theta2
        
        # Position after first link (at first elbow)
        # After base yaw and shoulder pitch, link 1 extends
        x1 = L1 * np.cos(theta1) * np.cos(effective_pitch)
        y1 = L1 * np.sin(theta1) * np.cos(effective_pitch)
        z1 = L1 * np.sin(effective_pitch)
        positions.append(np.array([x1, y1, z1]))
        
        # Position after second link (at second elbow)
        # Elbow 1 subtracts from the pitch (positive elbow = bend more forward)
        pitch_after_elbow1 = effective_pitch - theta3
        x2 = x1 + L2 * np.cos(theta1) * np.cos(pitch_after_elbow1)
        y2 = y1 + L2 * np.sin(theta1) * np.cos(pitch_after_elbow1)
        z2 = z1 + L2 * np.sin(pitch_after_elbow1)
        positions.append(np.array([x2, y2, z2]))
        
        # End effector position (after third link)
        # Elbow 2 subtracts from the cumulative pitch (positive elbow = bend more forward)
        pitch_after_elbow2 = pitch_after_elbow1 - theta4
        x3 = x2 + L3 * np.cos(theta1) * np.cos(pitch_after_elbow2)
        y3 = y2 + L3 * np.sin(theta1) * np.cos(pitch_after_elbow2)
        z3 = z2 + L3 * np.sin(pitch_after_elbow2)
        positions.append(np.array([x3, y3, z3]))
        
        return positions
    
    def check_ground_collision(
        self, 
        angles: Optional[np.ndarray] = None
    ) -> Tuple[bool, List[str]]:
        """
        Check if any part of the arm is below the ground plane.
        
        Args:
            angles: Joint angles to check. Uses current angles if None.
            
        Returns:
            Tuple of (is_valid, list_of_violations)
        """
        positions = self.get_all_joint_positions(angles)
        ground_z = self.config.ground_plane_z
        
        violations = []
        position_names = ["shoulder", "elbow_1", "elbow_2", "end_effector"]
        
        for i, pos in enumerate(positions):
            if pos[2] < ground_z:
                name = position_names[i] if i < len(position_names) else f"point_{i}"
                violations.append(
                    f"{name} at z={pos[2]:.3f} mm is below ground (z={ground_z} mm)"
                )
        
        is_valid = len(violations) == 0
        return is_valid, violations
    
    def get_state(
        self, 
        angles: Optional[np.ndarray] = None
    ) -> ArmState:
        """
        Get the complete state of the arm.
        
        Args:
            angles: Joint angles. Uses current angles if None.
            
        Returns:
            ArmState object with all state information
        """
        if angles is None:
            angles = self._current_angles
        else:
            angles = np.asarray(angles)
        
        # Compute forward kinematics
        pose = self.forward_kinematics(angles)
        
        # Get all joint positions
        positions = self.get_all_joint_positions(angles)
        
        # Check constraints
        violations = []
        
        # Check joint limits
        if not self.is_within_limits(angles):
            lower, upper = self.get_joint_limits()
            for i, (a, l, u) in enumerate(zip(angles, lower, upper)):
                if a < l or a > u:
                    violations.append(
                        f"Joint {i+1} at {np.degrees(a):.1f}° "
                        f"outside limits [{np.degrees(l):.1f}°, {np.degrees(u):.1f}°]"
                    )
        
        # Check ground collision
        if self.config.enable_ground_collision:
            _, ground_violations = self.check_ground_collision(angles)
            violations.extend(ground_violations)
        
        return ArmState(
            joint_angles=angles,
            end_effector_pose=pose,
            joint_positions=positions,
            is_valid=len(violations) == 0,
            constraint_violations=violations
        )
    
    def set_joint_angles(
        self, 
        angles: Union[np.ndarray, List[float]],
        enforce_limits: bool = True,
        check_ground: bool = True
    ) -> ArmState:
        """
        Set joint angles with optional constraint enforcement.
        
        Args:
            angles: Desired joint angles
            enforce_limits: Whether to clamp to joint limits
            check_ground: Whether to check ground collision
            
        Returns:
            Resulting ArmState after setting angles
        """
        angles = np.asarray(angles)
        
        if enforce_limits:
            angles = self.clamp_to_limits(angles)
        
        self._current_angles = angles
        state = self.get_state()
        
        # If ground collision detected and checking enabled, 
        # the state will have violations but we don't automatically fix
        # (that would require inverse kinematics)
        
        return state
    
    def move_joint(
        self, 
        joint_index: int, 
        delta: float,
        enforce_limits: bool = True
    ) -> ArmState:
        """
        Move a single joint by a delta amount.
        
        Args:
            joint_index: Index of joint to move (0-2)
            delta: Amount to move in radians
            enforce_limits: Whether to enforce joint limits
            
        Returns:
            Resulting ArmState
        """
        if not 0 <= joint_index < self.num_joints:
            raise ValueError(f"Joint index must be 0-{self.num_joints-1}")
        
        new_angles = self._current_angles.copy()
        new_angles[joint_index] += delta
        
        return self.set_joint_angles(new_angles, enforce_limits=enforce_limits)
    
    def go_home(self) -> ArmState:
        """
        Move arm to home position.
        
        Returns:
            ArmState at home position
        """
        return self.set_joint_angles(self.config.get_home_position())
    
    def get_jacobian(
        self, 
        angles: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Compute the Jacobian matrix at the given configuration.
        
        The Jacobian relates joint velocities to end effector velocities.
        
        Args:
            angles: Joint angles. Uses current angles if None.
            
        Returns:
            6xN Jacobian matrix (N = number of joints)
        """
        if angles is None:
            angles = self._current_angles
        return self.robot.jacob0(angles)
    
    def __repr__(self) -> str:
        """String representation of the arm."""
        return (
            f"ThreeLinkArm(name='{self.config.name}', "
            f"joints={self.num_joints}, "
            f"total_reach={self.total_reach:.1f} mm)"
        )
    
    def summary(self) -> str:
        """Get a detailed summary of the arm configuration."""
        lines = [
            f"{'='*50}",
            f"  {self.config.name}",
            f"{'='*50}",
            f"  Total Joints: {self.num_joints}",
            f"  Total Reach: {self.total_reach:.1f} mm",
            f"  Ground Collision Check: {'Enabled' if self.config.enable_ground_collision else 'Disabled'}",
            f"",
            f"  Joints:",
        ]
        
        for i, joint in enumerate(self.config.joints):
            lines.append(
                f"    [{i}] {joint.name}: "
                f"{np.degrees(joint.limit_lower):.0f}° to "
                f"{np.degrees(joint.limit_upper):.0f}° "
                f"(home: {np.degrees(joint.home_position):.0f}°)"
            )
        
        lines.append(f"")
        lines.append(f"  Links:")
        
        for i, link in enumerate(self.config.links):
            lines.append(f"    [{i}] {link.name}: {link.length:.1f} mm")
        
        lines.append(f"{'='*50}")
        
        return "\n".join(lines)


# =============================================================================
# Module-level convenience functions
# =============================================================================

def create_arm(config: Optional[RobotConfig] = None) -> ThreeLinkArm:
    """
    Factory function to create a ThreeLinkArm.
    
    Args:
        config: Robot configuration. Uses default if None.
        
    Returns:
        Configured ThreeLinkArm instance
    """
    return ThreeLinkArm(config)


def demo():
    """Demonstration of the ThreeLinkArm capabilities."""
    print("Creating 3-Link Robotic Arm (4 DOF)...")
    arm = ThreeLinkArm()
    
    print(arm.summary())
    
    # Test forward kinematics at home position
    print("\n--- Home Position ---")
    state = arm.go_home()
    print(f"Joint angles: {np.degrees(state.joint_angles)} degrees")
    print(f"End effector position: {arm.get_end_effector_position()}")
    print(f"Valid state: {state.is_valid}")
    
    # Test moving joints
    print("\n--- Moving Base Joint ---")
    arm.move_joint(0, np.pi/4)  # Rotate 45 degrees
    print(f"After 45° base rotation: {arm.get_end_effector_position()}")
    
    # Test joint limits
    print("\n--- Testing Joint Limits ---")
    arm.go_home()
    state = arm.set_joint_angles([0, -0.1, 0, 0])  # Try to go below shoulder limit
    print(f"Attempted to set shoulder below limit")
    print(f"Clamped angles: {np.degrees(state.joint_angles)} degrees")
    
    # Test elbow articulation
    print("\n--- Testing Elbow Joints ---")
    arm.go_home()
    state = arm.set_joint_angles([0, np.pi/2, np.pi/4, -np.pi/4])  # Both elbows bent
    print(f"Both elbows bent: {arm.get_end_effector_position()}")
    
    # Test ground collision
    print("\n--- Testing Ground Collision ---")
    arm.go_home()
    state = arm.set_joint_angles([0, np.pi/6, -np.pi/3, -np.pi/4])  # Might hit ground
    is_valid, violations = arm.check_ground_collision()
    print(f"Configuration valid: {is_valid}")
    if violations:
        for v in violations:
            print(f"  Violation: {v}")


if __name__ == "__main__":
    demo()
