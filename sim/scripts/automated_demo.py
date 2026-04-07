"""
Automated Demo Script for 3-Link Robotic Arm

This script runs a single comprehensive demo that highlights:
- Full workspace coverage (sweeps and reaches)
- Complex bending and folding
- Line/shape traces
- Joint motion with a mostly fixed end effector

Usage:
    pipenv run python sim/scripts/automated_demo.py

Author: Senior Design Team
Date: January 2026
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

_SCRIPT_DIR = Path(__file__).resolve().parent
_MODEL_DIR = _SCRIPT_DIR.parent / "model"
if str(_MODEL_DIR) not in sys.path:
    sys.path.insert(0, str(_MODEL_DIR))

from arm_model import ThreeLinkArm
from view_robot import ArmViewer, TrajectoryAnimator


def get_comprehensive_waypoints():
    """Build a single, rich trajectory that exercises the whole arm."""
    waypoints = []

    def add(*angles):
        waypoints.append(np.array(angles, dtype=float))

    # Phase 0: Home and gentle warmup
    add(0.0, 0.0, 0.0, 0.0)
    add(0.0, np.pi / 8, np.pi / 12, -np.pi / 12)
    add(0.0, np.pi / 6, np.pi / 8, 0.0)

    # Phase 1: Sweeps around the base with moderate extension
    for yaw in np.linspace(-np.pi / 1.1, np.pi / 1.1, 7):
        add(yaw, np.pi / 6, np.pi / 7, np.pi / 7)

    # Phase 2: Reach tests in cardinal directions and a fold
    add(0.0, np.pi / 3, np.pi / 8, np.pi / 8)             # forward
    add(np.pi / 2, np.pi / 3, np.pi / 8, np.pi / 8)       # right
    add(-np.pi / 2, np.pi / 3, np.pi / 8, np.pi / 8)      # left
    add(np.pi, np.pi / 3, np.pi / 8, np.pi / 8)           # backward
    add(0.0, 0.0, np.pi / 3, np.pi / 4)                   # upward curl
    add(0.0, -np.pi / 4, -np.pi / 5, np.pi / 5)           # back fold

    # Phase 3: Trace a forward-oriented rectangular path
    add(0.0, np.pi / 3, np.pi / 6, np.pi / 12)            # corner 1
    add(0.0, np.pi / 3, np.pi / 6, -np.pi / 18)           # slide down
    add(0.25, np.pi / 3, np.pi / 6, -np.pi / 18)          # slide right
    add(0.25, np.pi / 4, np.pi / 5, -np.pi / 18)          # drop
    add(-0.25, np.pi / 4, np.pi / 5, -np.pi / 18)         # slide left
    add(-0.25, np.pi / 3, np.pi / 6, np.pi / 18)          # rise
    add(0.0, np.pi / 3, np.pi / 6, np.pi / 12)            # close loop

    # Phase 4: Joint articulation while keeping the end effector nearly fixed
    anchor = np.array([np.pi / 4, np.pi / 3, np.pi / 7, -np.pi / 10])
    waypoints.append(anchor)
    for delta in np.linspace(-0.35, 0.35, 7):
        compensated = anchor + np.array([0.0, 0.12 * delta, -0.9 * delta, 0.8 * delta])
        waypoints.append(compensated)
    waypoints.append(anchor)

    # Phase 5: Tight bends and return home
    add(np.pi / 2, np.pi / 5, -np.pi / 3, np.pi / 5)
    add(-np.pi / 2, np.pi / 5, np.pi / 3, -np.pi / 5)
    add(0.0, 0.0, 0.0, 0.0)

    return waypoints


def run_automated_demo(steps_per_waypoint: int = 45, delay: float = 0.02):
    """Run the comprehensive automated demonstration."""
    print("=" * 60)
    print("  3-LINK ROBOTIC ARM - COMPREHENSIVE DEMO")
    print("=" * 60)
    print()
    print("This single demo covers sweeps, reaches, bends, traces, and joint-only motions.")
    print()

    arm = ThreeLinkArm()
    print(arm.summary())

    waypoints = get_comprehensive_waypoints()

    print(f"\nDemo Configuration:")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Steps per waypoint: {steps_per_waypoint}")
    print(f"  Total frames: ~{len(waypoints) * steps_per_waypoint}")
    print()

    print("Waypoint Preview:")
    print("-" * 50)
    for i, wp in enumerate(waypoints):
        pos = arm.get_all_joint_positions(wp)[-1]
        print(
            f"  [{i:02d}] Joints: {np.degrees(wp).astype(int)}° -> "
            f"End: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})"
        )
    print("-" * 50)
    print()

    viewer = ArmViewer(arm, title="3-Link Arm - Comprehensive Demo")
    animator = TrajectoryAnimator(arm, viewer)

    viewer._setup_figure()
    viewer._update_plot()

    print("Starting animation...")
    print("(Close the window to stop)")
    print()

    try:
        animator.animate_trajectory(
            waypoints,
            steps_per_waypoint=steps_per_waypoint,
            delay=delay,
        )

        print("\nAnimation complete! Window will stay open.")
        print("Close the window to exit.")
        plt.show()

    except KeyboardInterrupt:
        print("\nAnimation interrupted by user.")
        animator.stop()

    print("\nDemo finished. Goodbye!")


def main():
    """Main entry point."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Run the comprehensive automated demo for the 3-link arm"
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=45,
        help="Interpolation steps per waypoint (smoothness)",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.02,
        help="Delay between frames in seconds",
    )

    args = parser.parse_args()
    run_automated_demo(steps_per_waypoint=args.steps, delay=args.delay)


if __name__ == "__main__":
    main()
