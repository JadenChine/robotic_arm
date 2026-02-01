"""
Core Module

Contains core utilities and components including:
- Constraint system for joint limits and collision avoidance
- Utility functions

Usage:
    from simulation.core.constraints import ConstraintManager
"""

from .constraints import (
    Constraint,
    ConstraintType,
    ConstraintSeverity,
    ConstraintViolation,
    ConstraintCheckResult,
    JointLimitConstraint,
    GroundCollisionConstraint,
    WorkspaceBoundaryConstraint,
    ConstraintManager,
    create_default_constraint_manager,
)

__all__ = [
    'Constraint',
    'ConstraintType',
    'ConstraintSeverity',
    'ConstraintViolation',
    'ConstraintCheckResult',
    'JointLimitConstraint',
    'GroundCollisionConstraint',
    'WorkspaceBoundaryConstraint',
    'ConstraintManager',
    'create_default_constraint_manager',
]
