"""
Manual Control Script for 3-Link Robotic Arm

This script launches the interactive viewer in manual control mode,
allowing the user to:
- Adjust each joint angle using sliders
- See real-time updates of arm position
- View end effector coordinates
- Monitor constraint violations

Usage:
    pipenv run python sim/scripts/manual_control.py

Author: Senior Design Team  
Date: January 2026
"""

import sys
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
_MODEL_DIR = _SCRIPT_DIR.parent / "model"
if str(_MODEL_DIR) not in sys.path:
    sys.path.insert(0, str(_MODEL_DIR))

from arm_model import ThreeLinkArm
from view_robot import ArmViewer


def main():
    """Main entry point for manual control."""
    print("=" * 60)
    print("  3-LINK ROBOTIC ARM - MANUAL CONTROL MODE")
    print("=" * 60)
    print()
    
    # Create the arm
    arm = ThreeLinkArm()
    print(arm.summary())
    
    print("\nINSTRUCTIONS:")
    print("-" * 40)
    print("• Use the sliders to control each joint")
    print("• 'Home' button: Return to home position")
    print("• 'Zero All' button: Set all joints to zero")
    print("• 'Random' button: Generate random pose")
    print("• Rotate view by dragging with mouse")
    print("-" * 40)
    print()
    
    # Set up callback to log changes
    def on_update(angles):
        pos = arm.get_end_effector_position()
        state = arm.get_state()
        if not state.is_valid:
            print(f"⚠️  Constraint violation detected!")
    
    # Create and show viewer
    viewer = ArmViewer(arm, title="3-Link Arm - Manual Control")
    viewer.set_update_callback(on_update)
    
    print("Opening viewer window...")
    print("(Close the window to exit)")
    print()
    
    viewer.show()
    
    print("\nViewer closed. Goodbye!")


if __name__ == "__main__":
    main()
