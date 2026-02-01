"""
Constraints and Safety Module for Robotic Arms

This module provides constraint checking and enforcement for robotic arms,
including:
- Joint limit enforcement
- Ground collision detection
- Workspace boundary checking
- Self-collision detection (future)
- Velocity/acceleration limits (future)

The constraint system is designed to be:
- Modular and extensible
- Applicable to any arm configuration
- Usable for both validation and enforcement

Author: Jaden Chine
Date: January 31st 2026
"""

import numpy as np
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum
from abc import ABC, abstractmethod


class ConstraintType(Enum):
    """Types of constraints that can be applied."""
    JOINT_LIMIT = "joint_limit"
    GROUND_COLLISION = "ground_collision"
    WORKSPACE_BOUNDARY = "workspace_boundary"
    SELF_COLLISION = "self_collision"
    VELOCITY_LIMIT = "velocity_limit"
    CUSTOM = "custom"


class ConstraintSeverity(Enum):
    """Severity levels for constraint violations."""
    WARNING = "warning"      # Soft limit, can be overridden
    ERROR = "error"          # Hard limit, should not be violated
    CRITICAL = "critical"    # Safety-critical, must be enforced


@dataclass
class ConstraintViolation:
    """
    Represents a single constraint violation.
    
    Attributes:
        constraint_type: Type of constraint that was violated
        severity: Severity level of the violation
        message: Human-readable description
        joint_index: Index of affected joint (if applicable)
        value: The violating value
        limit: The limit that was exceeded
    """
    constraint_type: ConstraintType
    severity: ConstraintSeverity
    message: str
    joint_index: Optional[int] = None
    value: Optional[float] = None
    limit: Optional[float] = None


@dataclass
class ConstraintCheckResult:
    """
    Result of checking constraints on a configuration.
    
    Attributes:
        is_valid: True if no ERROR or CRITICAL violations
        violations: List of all violations found
        corrected_config: Suggested corrected configuration (if applicable)
    """
    is_valid: bool
    violations: List[ConstraintViolation] = field(default_factory=list)
    corrected_config: Optional[np.ndarray] = None
    
    @property
    def has_warnings(self) -> bool:
        """Check if there are any warning-level violations."""
        return any(v.severity == ConstraintSeverity.WARNING for v in self.violations)
    
    @property
    def has_errors(self) -> bool:
        """Check if there are any error-level violations."""
        return any(v.severity == ConstraintSeverity.ERROR for v in self.violations)
    
    @property
    def has_critical(self) -> bool:
        """Check if there are any critical violations."""
        return any(v.severity == ConstraintSeverity.CRITICAL for v in self.violations)


class Constraint(ABC):
    """Abstract base class for constraints."""
    
    @property
    @abstractmethod
    def constraint_type(self) -> ConstraintType:
        """Return the type of this constraint."""
        pass
    
    @property
    @abstractmethod
    def severity(self) -> ConstraintSeverity:
        """Return the severity level of this constraint."""
        pass
    
    @abstractmethod
    def check(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> List[ConstraintViolation]:
        """
        Check if the configuration violates this constraint.
        
        Args:
            joint_angles: Current joint configuration
            joint_positions: Positions of joints in workspace (optional)
            
        Returns:
            List of violations (empty if constraint satisfied)
        """
        pass
    
    @abstractmethod
    def correct(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> np.ndarray:
        """
        Correct a configuration to satisfy this constraint.
        
        Args:
            joint_angles: Current joint configuration
            joint_positions: Positions of joints in workspace (optional)
            
        Returns:
            Corrected joint configuration
        """
        pass


class JointLimitConstraint(Constraint):
    """
    Constraint that enforces joint angle limits.
    
    This is the most basic and essential constraint for any robot.
    """
    
    def __init__(
        self, 
        lower_limits: np.ndarray,
        upper_limits: np.ndarray,
        severity: ConstraintSeverity = ConstraintSeverity.ERROR
    ):
        """
        Initialize joint limit constraint.
        
        Args:
            lower_limits: Array of lower limits for each joint
            upper_limits: Array of upper limits for each joint
            severity: Severity level for violations
        """
        self._lower = np.asarray(lower_limits)
        self._upper = np.asarray(upper_limits)
        self._severity = severity
        
    @property
    def constraint_type(self) -> ConstraintType:
        return ConstraintType.JOINT_LIMIT
    
    @property
    def severity(self) -> ConstraintSeverity:
        return self._severity
    
    def check(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> List[ConstraintViolation]:
        violations = []
        
        for i, (angle, lower, upper) in enumerate(zip(joint_angles, self._lower, self._upper)):
            if angle < lower:
                violations.append(ConstraintViolation(
                    constraint_type=self.constraint_type,
                    severity=self._severity,
                    message=f"Joint {i} below lower limit: {np.degrees(angle):.1f}° < {np.degrees(lower):.1f}°",
                    joint_index=i,
                    value=angle,
                    limit=lower
                ))
            elif angle > upper:
                violations.append(ConstraintViolation(
                    constraint_type=self.constraint_type,
                    severity=self._severity,
                    message=f"Joint {i} above upper limit: {np.degrees(angle):.1f}° > {np.degrees(upper):.1f}°",
                    joint_index=i,
                    value=angle,
                    limit=upper
                ))
        
        return violations
    
    def correct(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> np.ndarray:
        return np.clip(joint_angles, self._lower, self._upper)


class GroundCollisionConstraint(Constraint):
    """
    Constraint that prevents any part of the arm from going below ground.
    
    The ground is defined as the XY plane at a specified Z coordinate.
    """
    
    def __init__(
        self, 
        ground_z: float = 0.0,
        severity: ConstraintSeverity = ConstraintSeverity.CRITICAL
    ):
        """
        Initialize ground collision constraint.
        
        Args:
            ground_z: Z-coordinate of the ground plane
            severity: Severity level for violations
        """
        self._ground_z = ground_z
        self._severity = severity
    
    @property
    def constraint_type(self) -> ConstraintType:
        return ConstraintType.GROUND_COLLISION
    
    @property
    def severity(self) -> ConstraintSeverity:
        return self._severity
    
    def check(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> List[ConstraintViolation]:
        if joint_positions is None:
            return []  # Cannot check without positions
        
        violations = []
        position_names = ["base", "joint_1", "joint_2", "end_effector"]
        
        for i, pos in enumerate(joint_positions):
            if pos[2] < self._ground_z:
                name = position_names[i] if i < len(position_names) else f"point_{i}"
                violations.append(ConstraintViolation(
                    constraint_type=self.constraint_type,
                    severity=self._severity,
                    message=f"{name} below ground: z={pos[2]:.3f} < {self._ground_z}",
                    value=pos[2],
                    limit=self._ground_z
                ))
        
        return violations
    
    def correct(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> np.ndarray:
        # Ground collision correction requires inverse kinematics
        # For now, just return the original angles
        # A proper implementation would adjust angles to lift the arm
        return joint_angles


class WorkspaceBoundaryConstraint(Constraint):
    """
    Constraint that keeps the end effector within a defined workspace.
    
    The workspace can be defined as a sphere, cylinder, or box.
    """
    
    def __init__(
        self,
        center: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        max_radius: float = 10.0,
        min_radius: float = 0.0,
        severity: ConstraintSeverity = ConstraintSeverity.WARNING
    ):
        """
        Initialize workspace boundary constraint.
        
        Args:
            center: Center of the workspace
            max_radius: Maximum distance from center
            min_radius: Minimum distance from center (for hollow workspaces)
            severity: Severity level for violations
        """
        self._center = np.array(center)
        self._max_radius = max_radius
        self._min_radius = min_radius
        self._severity = severity
    
    @property
    def constraint_type(self) -> ConstraintType:
        return ConstraintType.WORKSPACE_BOUNDARY
    
    @property
    def severity(self) -> ConstraintSeverity:
        return self._severity
    
    def check(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> List[ConstraintViolation]:
        if joint_positions is None or len(joint_positions) == 0:
            return []
        
        violations = []
        
        # Check end effector position (last position)
        end_effector = joint_positions[-1]
        distance = np.linalg.norm(end_effector - self._center)
        
        if distance > self._max_radius:
            violations.append(ConstraintViolation(
                constraint_type=self.constraint_type,
                severity=self._severity,
                message=f"End effector outside workspace: distance={distance:.3f} > {self._max_radius}",
                value=distance,
                limit=self._max_radius
            ))
        
        if distance < self._min_radius:
            violations.append(ConstraintViolation(
                constraint_type=self.constraint_type,
                severity=self._severity,
                message=f"End effector inside exclusion zone: distance={distance:.3f} < {self._min_radius}",
                value=distance,
                limit=self._min_radius
            ))
        
        return violations
    
    def correct(
        self, 
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> np.ndarray:
        # Workspace correction requires inverse kinematics
        return joint_angles


class ConstraintManager:
    """
    Manages a collection of constraints and provides unified checking.
    
    This class aggregates multiple constraints and provides methods to
    check them all at once, with proper priority ordering.
    """
    
    def __init__(self):
        """Initialize the constraint manager."""
        self._constraints: List[Constraint] = []
    
    def add_constraint(self, constraint: Constraint):
        """
        Add a constraint to the manager.
        
        Args:
            constraint: Constraint to add
        """
        self._constraints.append(constraint)
    
    def remove_constraint(self, constraint: Constraint):
        """
        Remove a constraint from the manager.
        
        Args:
            constraint: Constraint to remove
        """
        if constraint in self._constraints:
            self._constraints.remove(constraint)
    
    def clear_constraints(self):
        """Remove all constraints."""
        self._constraints.clear()
    
    @property
    def constraints(self) -> List[Constraint]:
        """Get list of all constraints."""
        return self._constraints.copy()
    
    def check_all(
        self,
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> ConstraintCheckResult:
        """
        Check all constraints.
        
        Args:
            joint_angles: Joint configuration to check
            joint_positions: Joint positions in workspace
            
        Returns:
            Combined result of all constraint checks
        """
        all_violations = []
        
        for constraint in self._constraints:
            violations = constraint.check(joint_angles, joint_positions)
            all_violations.extend(violations)
        
        # Determine if configuration is valid
        # (no ERROR or CRITICAL violations)
        is_valid = not any(
            v.severity in (ConstraintSeverity.ERROR, ConstraintSeverity.CRITICAL)
            for v in all_violations
        )
        
        # Attempt to generate corrected configuration
        corrected = joint_angles.copy()
        for constraint in self._constraints:
            corrected = constraint.correct(corrected, joint_positions)
        
        return ConstraintCheckResult(
            is_valid=is_valid,
            violations=all_violations,
            corrected_config=corrected if not np.array_equal(corrected, joint_angles) else None
        )
    
    def enforce_all(
        self,
        joint_angles: np.ndarray,
        joint_positions: Optional[List[np.ndarray]] = None
    ) -> np.ndarray:
        """
        Enforce all constraints by correcting the configuration.
        
        Args:
            joint_angles: Joint configuration to correct
            joint_positions: Joint positions in workspace
            
        Returns:
            Corrected joint configuration
        """
        corrected = joint_angles.copy()
        
        for constraint in self._constraints:
            corrected = constraint.correct(corrected, joint_positions)
        
        return corrected


def create_default_constraint_manager(
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    ground_z: float = 0.0,
    max_reach: float = 10.0
) -> ConstraintManager:
    """
    Create a constraint manager with default constraints for a typical arm.
    
    Args:
        lower_limits: Joint lower limits
        upper_limits: Joint upper limits
        ground_z: Ground plane Z coordinate
        max_reach: Maximum workspace radius
        
    Returns:
        Configured ConstraintManager
    """
    manager = ConstraintManager()
    
    # Add joint limits (ERROR severity - hard stops)
    manager.add_constraint(JointLimitConstraint(
        lower_limits, 
        upper_limits,
        severity=ConstraintSeverity.ERROR
    ))
    
    # Add ground collision (CRITICAL - safety)
    manager.add_constraint(GroundCollisionConstraint(
        ground_z,
        severity=ConstraintSeverity.CRITICAL
    ))
    
    # Add workspace boundary (WARNING - soft limit)
    manager.add_constraint(WorkspaceBoundaryConstraint(
        max_radius=max_reach,
        severity=ConstraintSeverity.WARNING
    ))
    
    return manager
