"""
Interactive 3D Viewer for Robotic Arms

This module provides visualization capabilities using matplotlib for 3D
interactive viewing of robotic arm configurations and movements.

Features:
- 3D visualization with XY ground plane and Z-up orientation
- Interactive joint control via sliders
- Real-time position updates
- Ground plane visualization
- Joint and link labeling
- Workspace visualization (optional)

Usage:
    from simulation.visualization.viewer import ArmViewer
    from simulation.models.three_link_arm import ThreeLinkArm
    
    arm = ThreeLinkArm()
    viewer = ArmViewer(arm)
    viewer.show()

Author: Senior Design Team
Date: January 2026
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from typing import Optional, List, Tuple, Callable
import sys
from pathlib import Path

# Add parent directory to path for imports
_module_dir = Path(__file__).parent.parent
if str(_module_dir) not in sys.path:
    sys.path.insert(0, str(_module_dir))


class ArmViewer:
    """
    Interactive 3D viewer for robotic arms.
    
    Provides matplotlib-based visualization with:
    - 3D arm rendering
    - Joint angle sliders
    - Real-time updates
    - Ground plane display
    - Position readouts
    """
    
    # Color scheme
    COLORS = {
        'link': '#2E86AB',          # Blue for links
        'joint': '#A23B72',          # Magenta for joints
        'end_effector': '#F18F01',   # Orange for end effector
        'ground': '#C5C5C5',         # Gray for ground
        'ground_grid': '#888888',    # Darker gray for grid
        'axis_x': '#E63946',         # Red for X axis
        'axis_y': '#2A9D8F',         # Green for Y axis
        'axis_z': '#457B9D',         # Blue for Z axis
        'warning': '#FFB703',        # Yellow for warnings
        'error': '#E63946',          # Red for errors
    }
    
    def __init__(
        self,
        arm,
        figsize: Tuple[int, int] = (14, 10),
        title: str = "3-Link Robotic Arm Viewer"
    ):
        """
        Initialize the viewer.
        
        Args:
            arm: ThreeLinkArm instance to visualize
            figsize: Figure size (width, height) in inches
            title: Window title
        """
        self.arm = arm
        self.figsize = figsize
        self.title = title
        
        # Visualization state
        self._fig = None
        self._ax = None
        self._sliders = []
        self._update_callback: Optional[Callable] = None
        
        # Plot elements (for updating)
        self._link_lines = None
        self._joint_points = None
        self._end_effector_point = None
        self._position_text = None
        self._status_text = None
        
    def _setup_figure(self):
        """Set up the matplotlib figure and axes."""
        # Create figure with specific layout
        self._fig = plt.figure(figsize=self.figsize)
        self._fig.suptitle(self.title, fontsize=14, fontweight='bold')
        
        # Main 3D axes (takes up most of the figure)
        self._ax = self._fig.add_axes([0.05, 0.25, 0.6, 0.7], projection='3d')
        
        # Configure 3D axes
        self._setup_3d_axes()
        
        # Add sliders
        self._setup_sliders()
        
        # Add info panel
        self._setup_info_panel()
        
        # Add buttons
        self._setup_buttons()
        
    def _setup_3d_axes(self):
        """Configure the 3D axes with proper orientation and labels."""
        ax = self._ax
        
        # Set axis limits based on arm reach
        reach = self.arm.total_reach * 1.2  # Add 20% margin
        ax.set_xlim(-reach, reach)
        ax.set_ylim(-reach, reach)
        ax.set_zlim(0, reach)  # Z starts at 0 (ground)
        
        # Labels
        ax.set_xlabel('X', fontsize=12, fontweight='bold', color=self.COLORS['axis_x'])
        ax.set_ylabel('Y', fontsize=12, fontweight='bold', color=self.COLORS['axis_y'])
        ax.set_zlabel('Z', fontsize=12, fontweight='bold', color=self.COLORS['axis_z'])
        
        # Set viewing angle: looking down at XY plane
        # X points to top-right, Y points to top-left, Z points up toward viewer
        ax.view_init(elev=25, azim=-135)
        
        # Draw ground plane
        self._draw_ground_plane()
        
        # Draw coordinate axes at origin
        self._draw_origin_axes()
        
    def _draw_ground_plane(self):
        """Draw the ground plane (XY plane at Z=0)."""
        ax = self._ax
        reach = self.arm.total_reach
        
        # Create grid for ground plane
        grid_size = int(reach)
        x = np.linspace(-reach, reach, grid_size * 2 + 1)
        y = np.linspace(-reach, reach, grid_size * 2 + 1)
        
        # Draw grid lines
        for xi in x:
            ax.plot([xi, xi], [-reach, reach], [0, 0], 
                   color=self.COLORS['ground_grid'], alpha=0.3, linewidth=0.5)
        for yi in y:
            ax.plot([-reach, reach], [yi, yi], [0, 0],
                   color=self.COLORS['ground_grid'], alpha=0.3, linewidth=0.5)
        
        # Draw ground plane surface (semi-transparent)
        xx, yy = np.meshgrid([-reach, reach], [-reach, reach])
        zz = np.zeros_like(xx)
        ax.plot_surface(xx, yy, zz, alpha=0.1, color=self.COLORS['ground'])
        
    def _draw_origin_axes(self):
        """Draw coordinate axes at the origin."""
        ax = self._ax
        axis_length = self.arm.total_reach * 0.15
        
        # X axis (red)
        ax.quiver(0, 0, 0, axis_length, 0, 0, color=self.COLORS['axis_x'], 
                 arrow_length_ratio=0.1, linewidth=2)
        # Y axis (green)
        ax.quiver(0, 0, 0, 0, axis_length, 0, color=self.COLORS['axis_y'],
                 arrow_length_ratio=0.1, linewidth=2)
        # Z axis (blue)
        ax.quiver(0, 0, 0, 0, 0, axis_length, color=self.COLORS['axis_z'],
                 arrow_length_ratio=0.1, linewidth=2)
        
    def _setup_sliders(self):
        """Set up joint angle sliders."""
        self._sliders = []
        
        slider_height = 0.03
        slider_spacing = 0.05
        slider_bottom = 0.08
        
        for i, joint in enumerate(self.arm.config.joints):
            # Calculate slider position
            ax_slider = self._fig.add_axes([
                0.1, 
                slider_bottom + (self.arm.num_joints - 1 - i) * slider_spacing,
                0.35,
                slider_height
            ])
            
            # Create slider
            slider = Slider(
                ax_slider,
                f'{joint.name}',
                np.degrees(joint.limit_lower),
                np.degrees(joint.limit_upper),
                valinit=np.degrees(joint.home_position),
                valstep=1.0,  # 1 degree steps
                color=self.COLORS['joint']
            )
            
            # Connect update function
            slider.on_changed(self._on_slider_changed)
            self._sliders.append(slider)
            
    def _setup_info_panel(self):
        """Set up the information panel on the right side."""
        # Position text area
        self._info_ax = self._fig.add_axes([0.68, 0.4, 0.3, 0.5])
        self._info_ax.axis('off')
        
        # Initial text
        self._position_text = self._info_ax.text(
            0.0, 0.95, '', 
            transform=self._info_ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            fontfamily='monospace'
        )
        
        self._status_text = self._info_ax.text(
            0.0, 0.3, '',
            transform=self._info_ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            fontfamily='monospace'
        )
        
    def _setup_buttons(self):
        """Set up control buttons."""
        # Home button
        ax_home = self._fig.add_axes([0.68, 0.25, 0.12, 0.05])
        self._btn_home = Button(ax_home, 'Home', color='lightgray')
        self._btn_home.on_clicked(self._on_home_clicked)
        
        # Zero button
        ax_zero = self._fig.add_axes([0.82, 0.25, 0.12, 0.05])
        self._btn_zero = Button(ax_zero, 'Zero All', color='lightgray')
        self._btn_zero.on_clicked(self._on_zero_clicked)
        
        # Random button
        ax_random = self._fig.add_axes([0.68, 0.18, 0.12, 0.05])
        self._btn_random = Button(ax_random, 'Random', color='lightgray')
        self._btn_random.on_clicked(self._on_random_clicked)
        
    def _on_slider_changed(self, val):
        """Handle slider value changes."""
        # Get angles from sliders (convert from degrees to radians)
        angles = np.array([np.radians(s.val) for s in self._sliders])
        
        # Update arm
        self.arm.set_joint_angles(angles, enforce_limits=True)
        
        # Update visualization
        self._update_plot()
        
        # Call external callback if set
        if self._update_callback:
            self._update_callback(self.arm.current_angles)
            
    def _on_home_clicked(self, event):
        """Handle home button click."""
        self.arm.go_home()
        self._sync_sliders_to_arm()
        self._update_plot()
        
    def _on_zero_clicked(self, event):
        """Handle zero button click."""
        # Set all joints to zero (clamped to limits)
        self.arm.set_joint_angles(np.zeros(self.arm.num_joints))
        self._sync_sliders_to_arm()
        self._update_plot()
        
    def _on_random_clicked(self, event):
        """Handle random button click."""
        # Generate random angles within limits
        lower, upper = self.arm.get_joint_limits()
        random_angles = np.random.uniform(lower, upper)
        self.arm.set_joint_angles(random_angles)
        self._sync_sliders_to_arm()
        self._update_plot()
        
    def _sync_sliders_to_arm(self):
        """Sync slider positions to current arm angles."""
        angles_deg = np.degrees(self.arm.current_angles)
        for slider, angle in zip(self._sliders, angles_deg):
            slider.set_val(angle)
            
    def _update_plot(self):
        """Update the 3D plot with current arm configuration."""
        ax = self._ax
        
        # Get joint positions
        positions = self.arm.get_all_joint_positions()
        state = self.arm.get_state()
        
        # Clear previous arm drawing (keep ground and axes)
        # Remove old elements if they exist
        if self._link_lines is not None:
            for line in self._link_lines:
                line.remove()
        if self._joint_points is not None:
            self._joint_points.remove()
        if self._end_effector_point is not None:
            self._end_effector_point.remove()
            
        # Extract coordinates
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        zs = [p[2] for p in positions]
        
        # Draw links
        self._link_lines = []
        for i in range(len(positions) - 1):
            line, = ax.plot(
                [xs[i], xs[i+1]], 
                [ys[i], ys[i+1]], 
                [zs[i], zs[i+1]],
                color=self.COLORS['link'],
                linewidth=4,
                solid_capstyle='round'
            )
            self._link_lines.append(line)
            
        # Draw joints
        self._joint_points = ax.scatter(
            xs[:-1], ys[:-1], zs[:-1],
            c=self.COLORS['joint'],
            s=100,
            marker='o',
            depthshade=True
        )
        
        # Draw end effector
        self._end_effector_point = ax.scatter(
            [xs[-1]], [ys[-1]], [zs[-1]],
            c=self.COLORS['end_effector'],
            s=150,
            marker='*',
            depthshade=True
        )
        
        # Update info text
        end_pos = positions[-1]
        info_text = (
            f"{'─' * 28}\n"
            f"  JOINT ANGLES\n"
            f"{'─' * 28}\n"
        )
        for i, (joint, angle) in enumerate(zip(self.arm.config.joints, self.arm.current_angles)):
            info_text += f"  {joint.name}:\n    {np.degrees(angle):7.1f}°\n"
        
        info_text += (
            f"\n{'─' * 28}\n"
            f"  END EFFECTOR\n"
            f"{'─' * 28}\n"
            f"  X: {end_pos[0]:8.3f}\n"
            f"  Y: {end_pos[1]:8.3f}\n"
            f"  Z: {end_pos[2]:8.3f}\n"
        )
        
        self._position_text.set_text(info_text)
        
        # Update status text
        if state.is_valid:
            status_text = "✓ Valid Configuration"
            status_color = 'green'
        else:
            status_text = "✗ Constraint Violations:\n"
            for v in state.constraint_violations[:3]:  # Show first 3
                status_text += f"  • {v}\n"
            status_color = self.COLORS['error']
            
        self._status_text.set_text(status_text)
        self._status_text.set_color(status_color)
        
        # Refresh
        self._fig.canvas.draw_idle()
        
    def set_update_callback(self, callback: Callable):
        """
        Set a callback function to be called on arm updates.
        
        Args:
            callback: Function taking joint angles array as argument
        """
        self._update_callback = callback
        
    def set_angles(self, angles: np.ndarray):
        """
        Programmatically set joint angles and update display.
        
        Args:
            angles: Joint angles in radians
        """
        self.arm.set_joint_angles(angles)
        self._sync_sliders_to_arm()
        self._update_plot()
        
    def show(self):
        """Show the interactive viewer."""
        self._setup_figure()
        self._update_plot()
        plt.show()
        
    def get_figure(self):
        """Get the matplotlib figure (for embedding)."""
        if self._fig is None:
            self._setup_figure()
            self._update_plot()
        return self._fig


class TrajectoryAnimator:
    """
    Animates the arm through a trajectory.
    
    Used for demonstrating automated movement sequences.
    """
    
    def __init__(self, arm, viewer: Optional[ArmViewer] = None):
        """
        Initialize the animator.
        
        Args:
            arm: ThreeLinkArm instance
            viewer: Optional ArmViewer to update during animation
        """
        self.arm = arm
        self.viewer = viewer
        self._is_running = False
        
    def animate_trajectory(
        self,
        waypoints: List[np.ndarray],
        steps_per_waypoint: int = 50,
        delay: float = 0.02
    ):
        """
        Animate through a series of waypoints.
        
        Args:
            waypoints: List of joint configurations to move through
            steps_per_waypoint: Number of interpolation steps between waypoints
            delay: Delay between frames in seconds
        """
        self._is_running = True
        
        for i in range(len(waypoints) - 1):
            if not self._is_running:
                break
                
            start = waypoints[i]
            end = waypoints[i + 1]
            
            # Linear interpolation
            for t in np.linspace(0, 1, steps_per_waypoint):
                if not self._is_running:
                    break
                    
                current = start + t * (end - start)
                self.arm.set_joint_angles(current)
                
                if self.viewer:
                    self.viewer._sync_sliders_to_arm()
                    self.viewer._update_plot()
                    plt.pause(delay)
                    
        self._is_running = False
        
    def stop(self):
        """Stop the animation."""
        self._is_running = False


def demo():
    """Demonstration of the viewer."""
    # Import here to avoid circular imports
    from models.three_link_arm import ThreeLinkArm
    
    print("Creating arm and viewer...")
    arm = ThreeLinkArm()
    viewer = ArmViewer(arm)
    
    print("Showing viewer (close window to exit)...")
    viewer.show()


if __name__ == "__main__":
    demo()
