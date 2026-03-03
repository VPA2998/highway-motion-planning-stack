"""
Visualization and Export

This module handles:
- Plotting the highway scenario (lanes, vehicles, trajectories)
- GIF animation generation
- CSV trajectory export
- Gradio interactive app
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
import csv
import os
import tempfile
from typing import List, Tuple, Dict, Optional, Any

LANEWIDTH = 3.7
VEHICLELENGTH = 4.5
VEHICLEWIDTH = 2.0

def lanecentery(laneindex):
    """Calculate center y-coordinate for a given lane."""
    return (laneindex - 1) * LANEWIDTH


def draw_lane_background(lmap: Any, ax) -> None:
    """Draw lane backgrounds and centerlines."""
    for lane in range(lmap.nlanes):
        y_center = lanecentery(lane)
        y_bottom = y_center - LANEWIDTH / 2
        
        # Lane rectangle
        rect = Rectangle(
            (0, y_bottom), lmap.length, LANEWIDTH,
            facecolor='lightgray', alpha=0.2, edgecolor='none'
        )
        ax.add_patch(rect)
        
        # Center line
        ax.axhline(y_center, color='gray', linestyle='--', linewidth=1, alpha=0.6)


def get_vehicle_corners(v: Any) -> np.ndarray:
    """Return 4 corners of vehicle rectangle."""
    from .collision import get_vehicle_corners as get_corners
    return get_corners(v)


def draw_vehicle_rect(ax, v: Any) -> None:
    """Draw vehicle as oriented rectangle."""
    corners = get_vehicle_corners(v)
    xmin, xmax = corners[:, 0].min(), corners[:, 0].max()
    ymin, ymax = corners[:, 1].min(), corners[:, 1].max()
    
    # Color: red for ego, blue for others
    color = 'D55E00' if v.is_ego else '0077BB'
    
    rect = Rectangle(
        (xmin, ymin), xmax - xmin, ymax - ymin,
        facecolor=f'#{color}', edgecolor='k', 
        alpha=0.9 if v.is_ego else 0.7
    )
    ax.add_patch(rect)


def draw_global_and_local(lmap: Any, vehicles: List, global_path_nodes: List,
                          ego_xy_hist: List, ego_lane_hist: List,
                          local_path: Optional[Any], behavior: Optional[str],
                          ax_global: Any, ax_local: Any) -> None:
    """
    Draw global and local views of the scenario.
    
    Args:
        lmap: LaneletMap object
        vehicles: List of vehicle objects
        global_path_nodes: A* path nodes
        ego_xy_hist: History of ego (x, y)
        ego_lane_hist: History of ego lane index
        local_path: FrenetTrajectory object
        behavior: Current FSM state string
        ax_global, ax_local: Matplotlib axes
    """
    ax_global.clear()
    ax_local.clear()
    
    # --- Global View ---
    draw_lane_background(lmap, ax_global)
    
    # Global route
    if global_path_nodes:
        xs, ys = [], []
        for node in global_path_nodes:
            x, y = lmap.nodetoxy(node)
            xs.append(x)
            ys.append(y)
        ax_global.plot(xs, ys, color='0077BB', linewidth=3, label='Global route')
    
    # Ego history
    if len(ego_xy_hist) > 1:
        xh = np.array([p[0] for p in ego_xy_hist])
        yh = np.array([p[1] for p in ego_xy_hist])
        ax_global.plot(xh, yh, color='black', linewidth=2.5, label='Ego history')
    
    # Local trajectory candidates
    if local_path is not None:
        xl, ybest = local_path.x, local_path.y
        candidates = local_path.get_candidates()
        best_idx = local_path.get_best_index()
        
        if candidates is not None:
            for i, cand in enumerate(candidates):
                yc = cand['d']
                xc = cand['x']
                n = min(len(xc), len(yc))
                
                if best_idx is not None and i == best_idx:
                    continue
                
                ax_global.plot(
                    xc[:n], yc[:n], color='gray', linestyle='--',
                    linewidth=1.0, alpha=0.6
                )
        
        ax_global.plot(
            xl, ybest, color='009E73', linestyle='--',
            linewidth=2, label='Local traj'
        )
    
    # Draw vehicles
    for v in vehicles:
        draw_vehicle_rect(ax_global, v)
    
    # Behavior text
    if behavior is not None:
        ax_global.text(
            0.02, 0.95, f'Behavior: {behavior}',
            transform=ax_global.transAxes,
            fontsize=11, verticalalignment='top', horizontalalignment='left',
            bbox=dict(boxstyle='round', pad=0.3, facecolor='white', alpha=0.8)
        )
    
    # Axis setup
    ax_global.set_xlim(0, lmap.length)
    ax_global.set_ylim(-2*LANEWIDTH, (lmap.nlanes - 1) * LANEWIDTH + 2*LANEWIDTH)
    ax_global.set_xlabel('x [m]')
    ax_global.set_ylabel('y [m]')
    ax_global.set_title('Global View: Route, Ego, Traffic, Local Trajectories')
    ax_global.grid(True, alpha=0.3)
    ax_global.legend(loc='lower right', fontsize=8)
    
    # --- Local Zoomed View ---
    if local_path is not None:
        xl, ybest = local_path.x, local_path.y
        candidates = local_path.get_candidates()
        best_idx = local_path.get_best_index()
        
        if candidates is not None:
            for i, cand in enumerate(candidates):
                yc = cand['d']
                xc = cand['x']
                n = min(len(xc), len(yc))
                
                if best_idx is not None and i == best_idx:
                    continue
                
                ax_local.plot(
                    xc[:n], yc[:n], color='gray', linestyle='--',
                    linewidth=1.0, alpha=0.6, label='cand'
                )
        
        ax_local.plot(
            xl, ybest, color='009E73', linewidth=3, label='Local traj'
        )
        ax_local.scatter(
            xl[0], ybest[0], c='D55E00', s=60, label='Start'
        )
        
        # Lane centerlines
        x0 = xl[0]
        x_local = np.linspace(x0 - 5, x0 + 60, 100)
        for lane in range(lmap.nlanes):
            yc = lanecentery(lane)
            ax_local.plot(
                x_local, np.ones_like(x_local) * yc,
                '--', color='gray', alpha=0.5, linewidth=1
            )
        
        ax_local.set_xlabel('x [m]')
        ax_local.set_ylabel('y [m]')
        ax_local.set_title('Local: Planned Trajectories (Zoomed)')
        ax_local.grid(True, alpha=0.3)
        ax_local.legend(fontsize=8)
        
        y0 = ybest[0]
        ax_local.set_xlim(x0 - 5, x0 + 60)
        ax_local.set_ylim(y0 - 2*LANEWIDTH, y0 + 2*LANEWIDTH)
        ax_local.set_aspect('equal', adjustable='box')


def build_gif(frames: List, fps: int = 10) -> str:
    """
    Build GIF from frames.
    
    Args:
        frames: List of PIL Image objects
        fps: Frames per second
    
    Returns:
        Path to saved GIF
    """
    tmp_dir = tempfile.mkdtemp()
    gif_path = os.path.join(tmp_dir, 'highway_sim_dynbicycle_rectcoll.gif')
    
    if frames:
        frames[0].save(
            gif_path,
            save_all=True,
            append_images=frames[1:],
            duration=int(1000 / fps),
            loop=0
        )
    
    return gif_path


def export_trajectory_csv(times: np.ndarray, ego_xy: np.ndarray, 
                          ego_lane: np.ndarray, ego_speed: np.ndarray,
                          behaviors: List[str]) -> str:
    """
    Export trajectory data to CSV.
    
    Columns: t, x, y, lane_index, speed, behavior
    
    Args:
        times: Time array
        ego_xy: Nx2 array of (x, y) positions
        ego_lane: Lane indices over time
        ego_speed: Speed over time
        behaviors: List of behavior states
    
    Returns:
        Path to saved CSV file
    """
    tmp_dir = tempfile.mkdtemp()
    csv_path = os.path.join(tmp_dir, 'ego_trajectory.csv')
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'y', 'lane_index', 'speed', 'behavior'])
        
        for t, (x, y), lane, v, b in zip(
            times, ego_xy, ego_lane, ego_speed, behaviors
        ):
            writer.writerow([
                float(t), float(x), float(y),
                float(lane), float(v), b
            ])
    
    return csv_path
