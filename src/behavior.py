"""
Behavior Planning: Finite State Machine (FSM)

This module handles:
- State definitions (CRUISE, FOLLOW, STOP, LANE_CHANGE_*)
- Transition cost matrix
- Traffic gap perception
- FSM logic for state selection
"""

from typing import Dict, List, Tuple, Optional, Any

# --- State Definitions ---
BEHAVIOR_STATES = [
    'CRUISE', 
    'FOLLOW', 
    'STOP', 
    'LANECHANGELEFT', 
    'LANECHANGERIGHT', 
    'ACCELERATE'
]

# --- Transition Cost Matrix ---
# Base cost to transition from one state to another.
# Lower = easier to transition. Higher = more resistance.
BASE_TRANSITION_COST = {s: {} for s in BEHAVIOR_STATES}

def set_cost(from_state: str, to_state: str, cost: float):
    """Helper to set transition cost."""
    BASE_TRANSITION_COST[from_state][to_state] = cost

# Initialize with a high default cost
for s_from in BEHAVIOR_STATES:
    for s_to in BEHAVIOR_STATES:
        BASE_TRANSITION_COST[s_from][s_to] = 10.0

# Define specific transition costs (copied from notebook)
# From CRUISE
set_cost('CRUISE', 'FOLLOW', 2.0)
set_cost('CRUISE', 'ACCELERATE', 2.0)
set_cost('CRUISE', 'LANECHANGELEFT', 3.0)
set_cost('CRUISE', 'LANECHANGERIGHT', 3.0)
set_cost('CRUISE', 'STOP', 5.0)
set_cost('CRUISE', 'CRUISE', 0.5) # Staying is cheap

# From FOLLOW
set_cost('FOLLOW', 'CRUISE', 2.0)
set_cost('FOLLOW', 'LANECHANGELEFT', 2.5)
set_cost('FOLLOW', 'LANECHANGERIGHT', 2.5)
set_cost('FOLLOW', 'STOP', 4.0)
set_cost('FOLLOW', 'FOLLOW', 0.5)

# From STOP
set_cost('STOP', 'CRUISE', 2.0)
set_cost('STOP', 'ACCELERATE', 1.5)
set_cost('STOP', 'STOP', 0.5)

# From Lane Change States
set_cost('LANECHANGELEFT', 'CRUISE', 2.0)
set_cost('LANECHANGELEFT', 'LANECHANGELEFT', 0.5)
set_cost('LANECHANGERIGHT', 'CRUISE', 2.0)
set_cost('LANECHANGERIGHT', 'LANECHANGERIGHT', 0.5)

# From ACCELERATE
set_cost('ACCELERATE', 'CRUISE', 1.5)
set_cost('ACCELERATE', 'FOLLOW', 3.0)
set_cost('ACCELERATE', 'ACCELERATE', 0.5)


# --- Traffic Perception ---

def compute_distances_to_vehicles(vehicles: List, ego: Any) -> Tuple[float, float, float, Optional[float]]:
    """
    Calculate gaps to the nearest vehicle in the same, left, and right lanes.
    
    Returns:
        Tuple: (d_same, d_left, d_right, lead_speed)
    """
    MAX_DIST = 1e6
    d_same = MAX_DIST
    d_left = MAX_DIST
    d_right = MAX_DIST
    lead_speed = None
    
    ego_lane_disc = int(round(ego.laneindex))
    
    for v in vehicles:
        if v.is_ego:
            continue
        
        # Only consider vehicles ahead
        if v.x > ego.x:
            gap = v.x - ego.x
            lane_v = int(round(v.laneindex))
            
            # Same lane
            if lane_v == ego_lane_disc and gap < d_same:
                d_same = gap
                lead_speed = v.speed
            
            # Left lane (lower index in our coordinate system? Check notebook logic)
            # Notebook: if lanev == egolanedisc - 1 -> left
            if lane_v == ego_lane_disc - 1 and gap < d_left:
                d_left = gap
            
            # Right lane
            # Notebook: if lanev == egolanedisc + 1 -> right
            if lane_v == ego_lane_disc + 1 and gap < d_right:
                d_right = gap
    
    return d_same, d_left, d_right, lead_speed


# --- Traffic Penalty Logic ---

SAFE_FOLLOW_DIST = 25.0
SAFE_LANE_CHANGE_DIST = 30.0

def traffic_penalty(current: str, next_state: str, 
                    d_same: float, d_left: float, d_right: float, 
                    lane_index: int, desired_lane: int) -> float:
    """
    Calculate penalty for a state transition based on traffic context.
    Lower penalty = safer/better transition.
    """
    penalty = 0.0
    
    # Immediate collision risk
    if d_same < 5.0 and next_state != 'STOP':
        penalty += 50.0
    
    # FOLLOW state logic
    if next_state == 'FOLLOW':
        if d_same <= SAFE_FOLLOW_DIST:
            penalty += 0.0 # Good to follow
        else:
            penalty += 5.0 # Unnecessary following
    
    # CRUISE logic
    if next_state == 'CRUISE' and d_same < SAFE_FOLLOW_DIST:
        penalty += 10.0 # Risky to cruise if too close
    
    # ACCELERATE logic
    if next_state == 'ACCELERATE' and d_same < SAFE_FOLLOW_DIST:
        penalty += 20.0 # Dangerous to accelerate
    
    # Lane Change Left
    if next_state == 'LANECHANGELEFT':
        if lane_index == 0: # Already in leftmost lane (assuming 0 is left)
            penalty += 100.0
        if d_left < SAFE_LANE_CHANGE_DIST:
            penalty += 30.0
        if lane_index != desired_lane:
            penalty -= 3.0 # Incentive to move towards goal lane
    
    # Lane Change Right
    if next_state == 'LANECHANGERIGHT':
        if lane_index == 2: # Already in rightmost lane (assuming 2 is right for 3-lane)
            penalty += 100.0
        if d_right < SAFE_LANE_CHANGE_DIST:
            penalty += 30.0
        if lane_index != desired_lane:
            penalty -= 3.0
    
    # Hysteresis: Slight penalty for changing state unnecessarily
    if next_state != current:
        penalty += 0.5
    
    return penalty


# --- FSM Class ---

class BehaviorFSM:
    """
    Finite State Machine for highway behavior.
    Inherits from a generic SM class structure or implements directly.
    """
    
    def __init__(self):
        self.state = 'CRUISE' # Start state
        self.lane_change_active = False
        self.current_lc_target_lane: Optional[int] = None
    
    def start(self):
        """Initialize/Reset FSM."""
        self.state = 'CRUISE'
        self.lane_change_active = False
        self.current_lc_target_lane = None
    
    def step(self, d_same: float, d_left: float, d_right: float, 
             lane: int, desired_lane: int, lc_complete: bool = False) -> str:
        """
        Execute one step of the FSM.
        
        Args:
            d_same, d_left, d_right: Gaps to traffic
            lane: Current lane index
            desired_lane: Goal lane from route planner
            lc_complete: Flag indicating if a lane change maneuver is finished
        
        Returns:
            New behavior state (str)
        """
        # Handle Lane Change Completion
        if self.lane_change_active and self.state in ['LANECHANGELEFT', 'LANECHANGERIGHT']:
            if lc_complete:
                self.lane_change_active = False
                self.current_lc_target_lane = None
                return 'CRUISE'
            else:
                return self.state # Stay in LC state
        
        # Evaluate all possible next states
        best_state = self.state
        best_cost = float('inf')
        
        for next_s in BEHAVIOR_STATES:
            # Base transition cost
            base_cost = BASE_TRANSITION_COST[self.state].get(next_s, 10.0)
            
            # Contextual penalty
            penalty = traffic_penalty(
                self.state, next_s, 
                d_same, d_left, d_right, 
                lane, desired_lane
            )
            
            total_cost = base_cost + penalty
            
            if total_cost < best_cost:
                best_cost = total_cost
                best_state = next_s
        
        # Activate lane change flags if selected
        if best_state == 'LANECHANGELEFT':
            self.lane_change_active = True
            self.current_lc_target_lane = max(0, lane - 1)
        elif best_state == 'LANECHANGERIGHT':
            self.lane_change_active = True
            self.current_lc_target_lane = min(2, lane + 1) # Assuming max 3 lanes
        
        self.state = best_state
        return best_state
