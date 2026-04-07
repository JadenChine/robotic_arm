#!/usr/bin/env python
"""
Test script for the 3-link robotic arm model.

Validates:
- Forward kinematics
- Joint limits
- Ground collision detection
- Position calculations

Usage:
    pipenv run python sim/scripts/test_arm.py
"""

import sys
from pathlib import Path

import numpy as np

_SCRIPT_DIR = Path(__file__).resolve().parent
_MODEL_DIR = _SCRIPT_DIR.parent / "model"
if str(_MODEL_DIR) not in sys.path:
    sys.path.insert(0, str(_MODEL_DIR))

from arm_model import ThreeLinkArm


def test_home_position():
    """Test that home position is correctly configured."""
    print("\n=== Test: Home Position ===")
    arm = ThreeLinkArm()
    arm.go_home()
    
    pos = arm.get_end_effector_position()
    expected = np.array([0, 0, 364])
    
    print(f"End effector at home: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"Expected: (0, 0, 364)")
    
    if np.allclose(pos, expected, atol=1e-6):
        print("✓ PASSED")
        return True
    else:
        print("✗ FAILED")
        return False


def test_horizontal_position():
    """Test horizontal arm configuration."""
    print("\n=== Test: Horizontal Position ===")
    arm = ThreeLinkArm()
    
    # Set to horizontal (shoulder at +90°, all elbows at 0)
    # Note: shoulder limit is -π/2 to π/2, so +π/2 is forward horizontal
    arm.set_joint_angles([0, np.pi/2, 0, 0])
    
    pos = arm.get_end_effector_position()
    expected = np.array([364, 0, 0])
    
    print(f"End effector horizontal: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"Expected: (364, 0, 0)")
    
    if np.allclose(pos, expected, atol=1e-6):
        print("✓ PASSED")
        return True
    else:
        print("✗ FAILED")
        return False


def test_base_rotation():
    """Test base rotation affects XY position correctly."""
    print("\n=== Test: Base Rotation ===")
    arm = ThreeLinkArm()
    
    # Arm horizontal (shoulder at +90°), rotated 90° around Z
    arm.set_joint_angles([np.pi/2, np.pi/2, 0, 0])
    
    pos = arm.get_end_effector_position()
    expected = np.array([0, 364, 0])
    
    print(f"End effector (base=90°): ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
    print(f"Expected: (0, 364, 0)")
    
    if np.allclose(pos, expected, atol=1e-6):
        print("✓ PASSED")
        return True
    else:
        print("✗ FAILED")
        return False


def test_joint_limits():
    """Test that joint limits are enforced."""
    print("\n=== Test: Joint Limits ===")
    arm = ThreeLinkArm()
    
    # Try to set angles beyond limits (above +90°)
    arm.set_joint_angles([0, 2.0, 0, 0])  # Shoulder above +π/2
    
    clamped = arm.current_angles
    print(f"Attempted shoulder angle: 2.0 rad (115°)")
    print(f"Clamped shoulder angle: {clamped[1]:.4f} rad ({np.degrees(clamped[1]):.1f}°)")
    print(f"Upper limit: +π/2 rad (+90°)")
    
    if np.isclose(clamped[1], np.pi/2, atol=1e-6):
        print("✓ PASSED")
        return True
    else:
        print("✗ FAILED")
        return False


def test_ground_collision():
    """Test ground collision detection."""
    print("\n=== Test: Ground Collision Detection ===")
    arm = ThreeLinkArm()
    ground_z = arm.config.ground_plane_z
    
    # Configuration that might put end effector below ground
    # With shoulder tilted forward and elbows bent forward (toward ground)
    # shoulder_pitch = +π/3 (60° forward from up), elbows bent forward
    arm.set_joint_angles([0, np.pi/3, np.pi/3, np.pi/4])
    
    is_valid, violations = arm.check_ground_collision()
    positions = arm.get_all_joint_positions()
    
    print(f"Joint positions (z values in mm):")
    for i, pos in enumerate(positions):
        name = ['Shoulder', 'Elbow 1', 'Elbow 2', 'End Effector'][i]
        print(f"  {name}: z = {pos[2]:.4f}")
    
    print(f"Ground plane z-limit: {ground_z} mm")
    print(f"Valid (no ground collision): {is_valid}")
    if not is_valid:
        for v in violations:
            print(f"  Violation: {v}")
    
    # Check if any z below allowed ground plane
    any_below = any(pos[2] < ground_z for pos in positions)
    
    if any_below and not is_valid:
        print("✓ PASSED (collision correctly detected)")
        return True
    elif not any_below and is_valid:
        print("✓ PASSED (no collision, correctly validated)")
        return True
    else:
        print("✗ FAILED")
        return False


def test_fk_consistency():
    """Test that RTB FK and manual calculation give same results."""
    print("\n=== Test: FK Consistency ===")
    arm = ThreeLinkArm()
    
    # Test several configurations (4 joints now)
    # shoulder_pitch: 0 = up, +π/2 = forward, -π/2 = backward
    # elbow angles: positive = bend forward, negative = bend backward
    test_configs = [
        [0, 0, 0, 0],                         # Home position (up)
        [np.pi/4, np.pi/6, np.pi/6, np.pi/6], # Tilted forward with elbows
        [-np.pi/4, 0, np.pi/4, -np.pi/4],     # Straight up with elbow bends
        [np.pi, np.pi/4, np.pi/4, np.pi/4],   # Rotated with shoulder forward
    ]
    
    all_passed = True
    for angles in test_configs:
        arm.set_joint_angles(angles)
        
        rtb_pos = arm.get_end_effector_position()
        manual_positions = arm.get_all_joint_positions()
        manual_pos = manual_positions[-1]
        
        match = np.allclose(rtb_pos, manual_pos, atol=1e-4)
        
        print(f"Config {np.degrees(angles).astype(int)}°:")
        print(f"  RTB:    ({rtb_pos[0]:.4f}, {rtb_pos[1]:.4f}, {rtb_pos[2]:.4f})")
        print(f"  Manual: ({manual_pos[0]:.4f}, {manual_pos[1]:.4f}, {manual_pos[2]:.4f})")
        print(f"  Match: {'✓' if match else '✗'}")
        
        if not match:
            all_passed = False
    
    if all_passed:
        print("\n✓ ALL PASSED")
    else:
        print("\n✗ SOME FAILED")
    
    return all_passed


def test_arm_summary():
    """Test that arm summary works correctly."""
    print("\n=== Test: Arm Summary ===")
    arm = ThreeLinkArm()
    print(arm.summary())
    print("✓ PASSED")
    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("  3-LINK ARM MODEL TESTS")
    print("=" * 60)
    
    tests = [
        test_arm_summary,
        test_home_position,
        test_horizontal_position,
        test_base_rotation,
        test_joint_limits,
        test_ground_collision,
        test_fk_consistency,
    ]
    
    results = []
    for test in tests:
        try:
            results.append(test())
        except Exception as e:
            print(f"\n✗ EXCEPTION: {e}")
            results.append(False)
    
    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    
    passed = sum(results)
    total = len(results)
    
    print(f"\nPassed: {passed}/{total}")
    
    if passed == total:
        print("\n✓ ALL TESTS PASSED")
        return 0
    else:
        print("\n✗ SOME TESTS FAILED")
        return 1


if __name__ == "__main__":
    sys.exit(main())
