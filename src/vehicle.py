"""
Vehicle Dynamics and Scenario Management

This module handles:
- Vehicle class (state, kinematics)
- Dynamic bicycle model for ego vehicle
- Kinematic update for other vehicles
- Scenario creation (ego + traffic)
"""

import numpy as np
from typing import List, Tuple, Optional

# Import global constants if you move them to a config file later
# For now, we define them here or import from a shared config
LANEWIDTH = 3.7
VEHICLELENGTH = 4.5
VEHICLEWIDTH = 2.0

def lanecentery(laneindex):
    """Calculate center y-coordinate for a given lane."""
    return (laneindex - 1) * LANEWIDTH


class Vehicle:
    """
    Represents a vehicle in the simulation.
    
    Attributes:
        laneindex: Logical lane index (0, 1, 2...)
        s: Frenet s-coordinate (longitudinal position)
        speed: Longitudinal speed (vx for ego)
        is_ego: Boolean flag for ego vehicle
        x, y: World coordinates
        yaw: Heading angle (radians)
        vx, vy, r: Dynamic bicycle states (longitudinal, lateral vel, yaw rate)
    """
    
    def __init__(self, laneindex, s, speed, is_ego=False):
        self.laneindex = float(laneindex)
        self.s = float(s)
        self.speed = float(speed)
        self.is_ego = is_ego
        
        # Physical pose
        self.x = s
        self.y = lanecentery(laneindex)
        self.yaw = 0.0  # rad, 0 for straight road
        
        # Dynamic bicycle states (primarily for ego)
        self.vx = max(speed, 0.1)
        self.vy = 0.0
        self.r = 0.0  # yaw rate

    def vehicle_xyv(self):
        """Return (x, y, v) tuple."""
        return self.x, self.y, self.speed


def get_vehicle_corners(v: Vehicle) -> np.ndarray:
    """
    Return 4 corners of oriented rectangle in world coords (x,y).
    Used for collision detection and visualization.
    """
    x, y = v.x, v.y
    L, W = VEHICLELENGTH, VEHICLEWIDTH
    yaw = v.yaw
    
    # Corners in local vehicle frame
    corners_local = np.array([
        [-L/2, -W/2],
        [ L/2, -W/2],
        [ L/2,  W/2],
        [-L/2,  W/2]
    ])
    
    # Rotation matrix
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    
    # Rotate and translate
    rotated = (R @ corners_local.T).T
    rotated[:, 0] += x
    rotated[:, 1] += y
    
    return rotated


def create_highway_scenario(lmap, num_vehicles, ego_lane, ego_s, ego_speed, seed=None):
    """
    Create a highway scenario with ego and random traffic.
    
    Args:
        lmap: LaneletMap object
        num_vehicles: Number of other vehicles
        ego_lane: Starting lane for ego
        ego_s: Starting s-coordinate for ego
        ego_speed: Initial speed for ego
        seed: Random seed for reproducibility
    
    Returns:
        Tuple: (list of all vehicles, ego vehicle reference)
    """
    if seed is not None:
        np.random.seed(seed)
        import random
        random.seed(seed)
    
    vehicles = []
    
    # Create ego
    ego = Vehicle(float(ego_lane), ego_s, ego_speed, is_ego=True)
    vehicles.append(ego)
    
    # Create other vehicles
    for _ in range(num_vehicles):
        lane = np.random.randint(0, lmap.nlanes)
        s = np.random.uniform(0.0, lmap.length)
        speed = np.random.uniform(8.0, 30.0)
        v = Vehicle(lane, s, speed, is_ego=False)
        vehicles.append(v)
    
    return vehicles, ego


# --- Dynamic Bicycle Model Parameters ---
M = 1500.0       # mass (kg)
IZ = 2500.0      # yaw inertia (kg m^2)
LF = 1.2         # distance from CG to front axle (m)
LR = 1.6         # distance from CG to rear axle (m)
CF = 80000.0     # cornering stiffness front (N/rad)
CR = 80000.0     # cornering stiffness rear (N/rad)
L = LF + LR


def stanley_steer(ego: Vehicle, target_lane: int) -> float:
    """
    Stanley-like steering to drive ego toward the center of target lane.
    For straight road, reference heading is 0 and reference y is lane center.
    """
    y_ref = lanecentery(target_lane)
    ey = y_ref - ego.y  # lateral error (positive if ego below ref)
    epsi = -ego.yaw     # heading error (ref heading is 0)
    
    k = 1.5
    v = max(ego.vx, 0.1)
    
    delta = epsi + np.arctan2(k * ey, v)
    
    max_steer = np.deg2rad(25.0)
    delta = np.clip(delta, -max_steer, max_steer)
    
    return delta


def update_ego_dynamics(ego: Vehicle, behavior: str, dt: float, 
                        lead_speed: Optional[float], dsame: float, 
                        target_lane: int, v0_desired: float, ax_idm: float):
    """
    Update ego using a simplified dynamic bicycle model.
    
    States: x, y, yaw, vx, vy, r
    Controls:
      - ax: longitudinal acceleration (from IDM)
      - delta: steering angle (from Stanley)
    """
    # Steering from Stanley toward target lane center
    delta = stanley_steer(ego, target_lane)
    
    vx = max(ego.vx, 0.1)
    vy = ego.vy
    r = ego.r
    psi = ego.yaw
    
    # Linear tire model (small angles)
    alpha_f = delta - (vy + LF * r) / vx
    alpha_r = -(vy - LR * r) / vx
    
    Fyf = CF * alpha_f
    Fyr = CR * alpha_r
    
    # Vehicle-body frame accelerations
    ax_total = ax_idm - (Fyf * np.sin(delta)) / M + vy * r
    ay = (Fyf * np.cos(delta) + Fyr) / M - vx * r
    
    # Yaw rate dynamics
    r_dot = (LF * Fyf * np.cos(delta) - LR * Fyr) / IZ
    
    # Integrate velocities
    vx = vx + ax_total * dt
    vy = vy + ay * dt
    vx = np.clip(vx, 0.0, 40.0)
    
    # Integrate yaw
    r = r + r_dot * dt
    psi = psi + r * dt
    
    # Integrate position in world frame
    x_dot = vx * np.cos(psi) - vy * np.sin(psi)
    y_dot = vx * np.sin(psi) + vy * np.cos(psi)
    
    ego.x = ego.x + x_dot * dt
    ego.y = ego.y + y_dot * dt
    ego.s = ego.x  # On straight road, s ~= x
    
    # Update state
    ego.vx = vx
    ego.vy = vy
    ego.r = r
    ego.yaw = psi
    ego.speed = vx  # Keep speed alias for logging
    
    return ego


def update_other_vehicles_physics(vehicles: List[Vehicle], dt: float, 
                                  v0_idm=25.0, amax_idm=1.5, b_idm=3.0, 
                                  s0_idm=2.0, T_idm=1.5):
    """
    Update non-ego vehicles with IDM per lane and prevent overlap.
    Simple kinematic update based on lane-following.
    """
    from .idm import idm_acceleration  # We will create this next!
    
    # Group by lane
    lanes = {}
    for v in vehicles:
        if v.is_ego:
            continue
        lane_disc = int(round(v.laneindex))
        lanes.setdefault(lane_disc, []).append(v)
    
    # Update each lane
    for lane, vs in lanes.items():
        # Sort by x position
        vs.sort(key=lambda v: v.x)
        
        for i, v in enumerate(vs):
            if i < len(vs) - 1:
                # Leader exists
                lead = vs[i+1]
                s_gap = max(lead.x - v.x - VEHICLELENGTH, 0.1)
                v_lead = lead.speed
            else:
                # No leader
                s_gap = np.inf
                v_lead = None
            
            # IDM acceleration
            a = idm_acceleration(v.speed, s_gap, v_lead, 
                                 v0=v0_idm, amax=amax_idm, b=b_idm, 
                                 s0=s0_idm, T=T_idm)
            
            # Euler integration
            a = np.clip(a, -4.0, 2.0)
            dv = np.clip(a * dt, -3.0 * dt, 3.0 * dt)
            v.speed = max(0.0, v.speed + dv)
            
            # Update position
            v.x = v.x + v.speed * dt
            v.s = v.x
            v.y = lanecentery(v.laneindex)
            v.yaw = 0.0
    
    # Prevent overlap (simple positional correction)
    for lane, vs in lanes.items():
        vs.sort(key=lambda v: v.x)
        for i in range(len(vs) - 1):
            lead = vs[i+1]
            foll = vs[i]
            if lead.x - foll.x < VEHICLELENGTH:
                foll.x = lead.x - VEHICLELENGTH
                foll.s = foll.x
                foll.speed = min(foll.speed, lead.speed)
