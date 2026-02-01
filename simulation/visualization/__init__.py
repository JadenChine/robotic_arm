"""
Visualization Module

Contains visualization tools for robotic arms:
- ArmViewer: Interactive 3D matplotlib viewer
- TrajectoryAnimator: Animation of trajectories

Usage:
    from simulation.visualization.view_robot import ArmViewer
    
    viewer = ArmViewer(arm)
    viewer.show()
"""

from .view_robot import ArmViewer, TrajectoryAnimator

__all__ = ['ArmViewer', 'TrajectoryAnimator']
