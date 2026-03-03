"""
Frenet Lattice Planner

Generates optimal lateral trajectories using quintic polynomials
in the Frenet frame (relative to the road centerline).

Features:
- Multi-lane, multi-horizon candidate generation
- Jerk minimization (smoothness)
- Collision risk assessment
- Route deviation penalty
"""

import numpy as np
from typing import List, Dict, Tuple, Optional, Any

# Import global constants (define here or import from config)
LANEWIDTH = 3.7

def lanecentery(laneindex):
    """Helper to get lane center y-coordinate."""
    return (laneindex - 1) * LANEWIDTH


def quintic_coeffs(d0: float, v0: float, a0: float, 
                   dT: float, vT: float, aT: float, 
                   T: float) -> np.ndarray:
    """
    Calculate coefficients for a quintic polynomial.
    
    Polynomial: d(t) = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0
    
    Args:
        d0, v0, a0: Initial lateral position, velocity, acceleration
        dT, vT, aT: Target lateral position, velocity, acceleration at time T
        T: Time horizon
    
    Returns:
        Array of 6 coefficients [a5, a4, a3, a2, a1, a0]
    """
    # Matrix A and vector b for linear system A*x = b
    A = np.array([
        [0, 0, 0, 0, 0, 1],
        [T**5, T**4, T**3, T**2, T, 1],
        [0, 0, 0, 0, 1, 0],
        [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
        [0, 0, 0, 2, 0, 0],
        [20*T**3, 12*T**2, 6*T, 2, 0, 0]
    ], dtype=float)
    
    b = np.array([d0, dT, v0, vT, a0, aT], dtype=float)
    
    coeffs = np.linalg.solve(A, b)
    return coeffs


def sample_lattice_candidates(ego: Any, behavior: str, 
                              route_desired_lane: int, 
                              nlanes: int) -> List[Tuple[int, float]]:
    """
    Generate candidate (lane, time_horizon) pairs.
    
    Args:
        ego: Vehicle object (for current state)
        behavior: Current FSM state
        route_desired_lane: Goal lane from global route
        nlanes: Total number of lanes
    
    Returns:
        List of tuples: [(lane_index, time_horizon), ...]
    """
    current_lane_disc = int(round(ego.laneindex))
    
    # Start with current lane and route-desired lane
    lanes = {current_lane_disc, route_desired_lane}
    
    # Add lane change targets if active
    if behavior == 'LANECHANGELEFT' and current_lane_disc > 0:
        lanes.add(current_lane_disc - 1)
    if behavior == 'LANECHANGERIGHT' and current_lane_disc < nlanes - 1:
        lanes.add(current_lane_disc + 1)
    
    # Time horizons to try
    horizons = [3.0, 4.0, 5.0]
    
    candidates = []
    for ln in lanes:
        # Ensure lane is valid
        ln = max(0, min(nlanes - 1, ln))
        for T in horizons:
            candidates.append((ln, T))
    
    return candidates


def evaluate_lattice_trajectory(d_traj: np.ndarray, t: np.ndarray, 
                                ego: Any, vehicles: List, 
                                route_lane_now: int) -> float:
    """
    Calculate cost for a single trajectory candidate.
    
    Cost components:
    1. Jerk (smoothness)
    2. Collision risk
    3. Route deviation
    
    Args:
        d_traj: Lateral positions over time
        t: Time array
        ego: Ego vehicle object
        vehicles: List of other vehicles
        route_lane_now: Current desired lane from route
    
    Returns:
        Total cost (float)
    """
    dt = t[1] - t[0] if len(t) > 1 else 0.1
    
    # 1. Jerk Cost (Smoothness)
    v_lat = np.gradient(d_traj, dt)
    a_lat = np.gradient(v_lat, dt)
    j_lat = np.gradient(a_lat, dt)
    jerk_cost = np.sum(j_lat**2) * dt
    
    # 2. Collision Cost
    min_dist = 1e6
    for v_other in vehicles:
        if v_other.is_ego:
            continue
        
        # Simple linear prediction of other vehicle
        s_other = v_other.x + v_other.speed * t
        d_other = lanecentery(v_other.laneindex)
        
        # Longitudinal trajectory (decoupled, constant velocity assumption for eval)
        s_traj = ego.x + ego.vx * t
        
        # Distance over time
        dist = np.sqrt((s_traj - s_other)**2 + (d_traj - d_other)**2)
        min_dist = min(min_dist, float(np.min(dist)))
    
    collision_cost = 0.0
    if min_dist < 5.0:
        collision_cost = 1e6  # Hard collision penalty
    elif min_dist < 15.0:
        collision_cost = (15.0 - min_dist) * 100.0
    
    # 3. Route Deviation Cost
    # Penalize ending up in a lane different from the route
    final_lane_idx = int(round((d_traj[-1] / LANEWIDTH) + 1))
    lane_deviation_cost = abs(final_lane_idx - route_lane_now) * 50.0
    
    # 4. Time Horizon Preference (prefer ~4s)
    T = t[-1]
    horizon_cost = (T - 4.0)**2 * 5.0
    
    total_cost = jerk_cost + collision_cost + lane_deviation_cost + horizon_cost
    return total_cost


class FrenetTrajectory:
    """Container for the best planned trajectory."""
    
    def __init__(self, x: np.ndarray, y: np.ndarray, t: np.ndarray, 
                 target_lane: int, candidates: Optional[List] = None, 
                 best_index: Optional[int] = None):
        self.x = x
        self.y = y
        self.t = t
        self.target_lane = target_lane
        self.candidates = candidates or []
        self.best_index = best_index
    
    def get_candidates(self) -> Optional[List]:
        """Return all evaluated candidates for visualization."""
        return self.candidates
    
    def get_best_index(self) -> Optional[int]:
        """Return index of the best candidate."""
        return self.best_index


def compute_frenet_lattice_local(ego: Any, behavior: str, 
                                 vehicles: List, 
                                 route_desired_lane_fn: Any, 
                                 nlanes: int = 3, 
                                 horizon_max: float = 5.0, 
                                 dt: float = 0.2) -> FrenetTrajectory:
    """
    Main entry point: Compute the optimal Frenet lattice trajectory.
    
    Args:
        ego: Ego vehicle object
        behavior: Current FSM state
        vehicles: List of other vehicles
        route_desired_lane_fn: Function(s) to get desired lane at position s
        nlanes: Number of lanes
        horizon_max: Maximum time horizon
        dt: Time step
    
    Returns:
        FrenetTrajectory object containing the best path
    """
    # Determine current desired lane from route
    if callable(route_desired_lane_fn):
        route_lane_now = route_desired_lane_fn(ego.s)
    else:
        route_lane_now = int(round(ego.laneindex))
    
    # Generate candidates
    cand_specs = sample_lattice_candidates(ego, behavior, route_lane_now, nlanes)
    
    # Initial conditions (assume starting from lane center with 0 lateral vel/accel)
    d0 = lanecentery(int(round(ego.laneindex)))
    v0 = 0.0
    a0 = 0.0
    
    best_cost = float('inf')
    best_d = None
    best_lane = int(round(ego.laneindex))
    best_idx = None
    best_T = 4.0
    best_t = None
    
    candidates_data = []
    
    # Ego longitudinal velocity for prediction
    vx0 = max(ego.vx, 0.1)
    
    for idx, (lane_T, T) in enumerate(cand_specs):
        # Time array
        t = np.arange(0.0, T + 1e-6, dt)
        
        # Target conditions (stop at lane center with 0 lateral motion)
        dT = lanecentery(lane_T)
        vT = 0.0
        aT = 0.0
        
        # Generate polynomial
        coeffs = quintic_coeffs(d0, v0, a0, dT, vT, aT, T)
        a5, a4, a3, a2, a1, a0c = coeffs
        
        # Evaluate trajectory
        d_traj = a5 * t**5 + a4 * t**4 + a3 * t**3 + a2 * t**2 + a1 * t + a0c
        
        # Longitudinal profile (decoupled, constant velocity for simplicity)
        x_traj_cand = ego.x + vx0 * t
        
        # Store candidate data for visualization
        candidates_data.append({
            'lane': lane_T,
            'T': T,
            'd': d_traj,
            't': t,
            'x': x_traj_cand
        })
        
        # Evaluate cost
        cost = evaluate_lattice_trajectory(
            d_traj, t, ego, vehicles, route_lane_now
        )
        
        # Add mild preference for route lane and ~4s horizon
        cost += abs(lane_T - route_lane_now) * 5.0
        cost += (T - 4.0)**2 * 5.0
        
        if cost < best_cost:
            best_cost = cost
            best_d = d_traj
            best_lane = lane_T
            best_idx = idx
            best_T = T
            best_t = t
    
    # Construct best longitudinal trajectory on the same time grid
    if best_t is not None:
        x_traj = ego.x + vx0 * best_t
    else:
        # Fallback if no candidates (should not happen)
        x_traj = np.array([ego.x])
        best_t = np.array([0.0])
        best_d = np.array([d0])
    
    return FrenetTrajectory(
        x=x_traj,
        y=best_d,
        t=best_t,
        target_lane=best_lane,
        candidates=candidates_data,
        best_index=best_idx
    )
