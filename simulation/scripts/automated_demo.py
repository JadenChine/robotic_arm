"""
Automated Demo Script for 3-Link Robotic Arm

This script demonstrates the arm moving through a predefined sequence
of waypoints, showcasing:
- Smooth trajectory interpolation
- Full range of motion
- Constraint enforcement
- Coordinated joint movement

Usage:
    pipenv run python simulation/scripts/automated_demo.py

Author: Senior Design Team
Date: January 2026
"""

import sys
from pathlib import Path

# Add parent directories to path for imports
_script_dir = Path(__file__).parent
_sim_dir = _script_dir.parent
if str(_sim_dir) not in sys.path:
    sys.path.insert(0, str(_sim_dir))

import numpy as np
import matplotlib.pyplot as plt

from models.three_link_arm import ThreeLinkArm
from visualization.view_robot import ArmViewer, TrajectoryAnimator


# =============================================================================
# DEMO WAYPOINT SEQUENCES
# =============================================================================

def get_demo_waypoints_basic():
    """
    Basic demonstration waypoints showing range of motion.
    
    Returns:
        List of joint configurations [base_yaw, shoulder_pitch, elbow1_pitch, elbow2_pitch]
        Note: shoulder_pitch 0 = straight up, positive = forward (+X), negative = backward (-X)
    """
    return [
        # Start at home position (pointing straight up)
        np.array([0.0, 0.0, 0.0, 0.0]),
        
        # Bend forward (shoulder_pitch = +45°)
        np.array([0.0, np.pi/4, 0.0, 0.0]),
        
        # Extend first elbow forward
        np.array([0.0, np.pi/4, np.pi/4, 0.0]),
        
        # Bend second elbow backward
        np.array([0.0, np.pi/4, np.pi/4, -np.pi/4]),
        
        # Rotate base 90 degrees
        np.array([np.pi/2, np.pi/4, np.pi/4, -np.pi/4]),
        
        # Articulate both elbows
        np.array([np.pi/2, np.pi/4, -np.pi/4, np.pi/4]),
        
        # Straighten up
        np.array([np.pi/2, 0.0, 0.0, 0.0]),
        
        # Rotate base to -90 degrees
        np.array([-np.pi/2, 0.0, 0.0, 0.0]),
        
        # Full forward reach with both elbows
        np.array([-np.pi/2, np.pi/4, np.pi/6, np.pi/6]),
        
        # Return to home
        np.array([0.0, 0.0, 0.0, 0.0]),
    ]


def get_demo_waypoints_sweep():
    """
    Sweeping motion demonstrating workspace coverage.
    
    Returns:
        List of joint configurations
    """
    waypoints = []
    
    # Start at home
    waypoints.append(np.array([0.0, 0.0, 0.0, 0.0]))
    
    # Sweep around the base while extended
    # shoulder_pitch = +π/6 (30° forward from up)
    for angle in np.linspace(0, 2*np.pi, 9)[:-1]:  # Full rotation minus duplicate
        waypoints.append(np.array([angle, np.pi/6, np.pi/6, np.pi/6]))
    
    # Return to home
    waypoints.append(np.array([0.0, 0.0, 0.0, 0.0]))
    
    return waypoints


def get_demo_waypoints_reach_test():
    """
    Reach test demonstrating maximum extension in various directions.
    
    Returns:
        List of joint configurations
    """
    return [
        # Home
        np.array([0.0, 0.0, 0.0, 0.0]),
        
        # Maximum reach forward (+X direction)
        # shoulder_pitch = +π/3 (60° forward from up)
        np.array([0.0, np.pi/3, np.pi/8, np.pi/8]),  # Near horizontal
        
        # Maximum reach right (+Y direction when base at 90°)
        np.array([np.pi/2, np.pi/3, np.pi/8, np.pi/8]),
        
        # Maximum reach backward (-X direction)
        np.array([np.pi, np.pi/3, np.pi/8, np.pi/8]),
        
        # Maximum reach left (-Y direction)
        np.array([-np.pi/2, np.pi/3, np.pi/8, np.pi/8]),
        
        # Maximum reach up (Z direction)
        np.array([0.0, 0.0, 0.0, 0.0]),
        
        # Compact/retracted position (both elbows bent backward)
        np.array([0.0, 0.0, -np.pi/3, -np.pi/4]),
        
        # Home
        np.array([0.0, 0.0, 0.0, 0.0]),
    ]


def get_demo_waypoints_pick_place():
    """
    Simulated pick and place motion.
    
    Returns:
        List of joint configurations
    """
    return [
        # Home/ready position
        np.array([0.0, 0.0, 0.0, 0.0]),
        
        # Move to "pick" position (forward and down)
        # shoulder_pitch = +π/6 (30° forward from up)
        np.array([0.0, np.pi/6, np.pi/6, np.pi/6]),
        
        # "Pick" - lower slightly with elbow articulation
        # shoulder_pitch = +π/4 (45° forward from up)
        np.array([0.0, np.pi/4, np.pi/4, np.pi/6]),
        
        # Lift up
        np.array([0.0, 0.0, np.pi/6, np.pi/6]),
        
        # Rotate to "place" position
        np.array([np.pi/2, 0.0, np.pi/6, np.pi/6]),
        
        # Move to place position
        np.array([np.pi/2, np.pi/6, np.pi/6, np.pi/6]),
        
        # "Place" - lower
        np.array([np.pi/2, np.pi/4, np.pi/4, np.pi/6]),
        
        # Retract
        np.array([np.pi/2, 0.0, 0.0, 0.0]),
        
        # Return home
        np.array([0.0, 0.0, 0.0, 0.0]),
    ]


# =============================================================================
# DEMO MODES
# =============================================================================

DEMO_MODES = {
    'basic': {
        'name': 'Basic Range of Motion',
        'description': 'Demonstrates basic joint movements and range of motion',
        'waypoints': get_demo_waypoints_basic,
        'steps_per_waypoint': 40,
        'delay': 0.03,
    },
    'sweep': {
        'name': 'Workspace Sweep',
        'description': 'Sweeps around the workspace to show coverage',
        'waypoints': get_demo_waypoints_sweep,
        'steps_per_waypoint': 30,
        'delay': 0.02,
    },
    'reach': {
        'name': 'Reach Test',
        'description': 'Tests maximum reach in various directions',
        'waypoints': get_demo_waypoints_reach_test,
        'steps_per_waypoint': 50,
        'delay': 0.03,
    },
    'pick_place': {
        'name': 'Pick and Place',
        'description': 'Simulates a pick and place operation',
        'waypoints': get_demo_waypoints_pick_place,
        'steps_per_waypoint': 45,
        'delay': 0.025,
    },
}


def run_automated_demo(mode: str = 'basic'):
    """
    Run an automated demonstration.
    
    Args:
        mode: Demo mode to run ('basic', 'sweep', 'reach', 'pick_place')
    """
    if mode not in DEMO_MODES:
        print(f"Unknown mode '{mode}'. Available modes: {list(DEMO_MODES.keys())}")
        return
        
    demo_config = DEMO_MODES[mode]
    
    print("=" * 60)
    print(f"  3-LINK ROBOTIC ARM - AUTOMATED DEMO")
    print(f"  Mode: {demo_config['name']}")
    print("=" * 60)
    print()
    print(f"Description: {demo_config['description']}")
    print()
    
    # Create arm
    arm = ThreeLinkArm()
    print(arm.summary())
    
    # Get waypoints
    waypoints = demo_config['waypoints']()
    
    print(f"\nDemo Configuration:")
    print(f"  Waypoints: {len(waypoints)}")
    print(f"  Steps per waypoint: {demo_config['steps_per_waypoint']}")
    print(f"  Total frames: ~{len(waypoints) * demo_config['steps_per_waypoint']}")
    print()
    
    # Preview waypoints
    print("Waypoint Summary:")
    print("-" * 50)
    for i, wp in enumerate(waypoints):
        pos = arm.get_all_joint_positions(wp)[-1]  # End effector position
        print(f"  [{i}] Joints: {np.degrees(wp).astype(int)}° -> "
              f"End: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
    print("-" * 50)
    print()
    
    # Create viewer and animator
    viewer = ArmViewer(arm, title=f"3-Link Arm - {demo_config['name']}")
    animator = TrajectoryAnimator(arm, viewer)
    
    # Set up the figure
    viewer._setup_figure()
    viewer._update_plot()
    
    print("Starting animation...")
    print("(Close the window to stop)")
    print()
    
    # Run animation
    try:
        animator.animate_trajectory(
            waypoints,
            steps_per_waypoint=demo_config['steps_per_waypoint'],
            delay=demo_config['delay']
        )
        
        # Keep window open after animation
        print("\nAnimation complete! Window will stay open.")
        print("Close the window to exit.")
        plt.show()
        
    except KeyboardInterrupt:
        print("\nAnimation interrupted by user.")
        animator.stop()
        
    print("\nDemo finished. Goodbye!")


def interactive_demo_selector():
    """Interactive menu for selecting demo mode."""
    print("=" * 60)
    print("  3-LINK ROBOTIC ARM - AUTOMATED DEMO SELECTOR")
    print("=" * 60)
    print()
    print("Available Demo Modes:")
    print()
    
    modes = list(DEMO_MODES.keys())
    for i, mode in enumerate(modes, 1):
        config = DEMO_MODES[mode]
        print(f"  [{i}] {config['name']}")
        print(f"      {config['description']}")
        print()
    
    print(f"  [0] Exit")
    print()
    
    while True:
        try:
            choice = input("Select demo mode (0-{}): ".format(len(modes)))
            choice = int(choice)
            
            if choice == 0:
                print("Goodbye!")
                return
            elif 1 <= choice <= len(modes):
                run_automated_demo(modes[choice - 1])
                return
            else:
                print("Invalid choice. Please try again.")
        except ValueError:
            print("Please enter a number.")
        except KeyboardInterrupt:
            print("\nGoodbye!")
            return


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Run automated demo for 3-link robotic arm'
    )
    parser.add_argument(
        '--mode', '-m',
        choices=list(DEMO_MODES.keys()),
        default=None,
        help='Demo mode to run. If not specified, shows interactive menu.'
    )
    parser.add_argument(
        '--list', '-l',
        action='store_true',
        help='List available demo modes and exit'
    )
    
    args = parser.parse_args()
    
    if args.list:
        print("Available demo modes:")
        for mode, config in DEMO_MODES.items():
            print(f"  {mode}: {config['name']}")
        return
    
    if args.mode:
        run_automated_demo(args.mode)
    else:
        interactive_demo_selector()


if __name__ == "__main__":
    main()
