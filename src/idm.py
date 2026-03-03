"""
Intelligent Driver Model (IDM)

Longitudinal car-following model.
"""

import numpy as np


def idm_acceleration(v, s_gap, v_lead, v0=25.0, amax=1.5, b=3.0, s0=2.0, T=1.5, delta=4.0):
    """
    Calculate IDM acceleration.
    
    Args:
        v: Ego velocity
        s_gap: Gap to leading vehicle
        v_lead: Leading vehicle velocity
        v0: Desired speed
        amax: Maximum acceleration
        b: Comfortable deceleration
        s0: Minimum gap
        T: Time gap
        delta: Acceleration exponent
    
    Returns:
        Acceleration (float)
    """
    v = max(v, 0.0)
    
    if s_gap <= 0.0:
        return -b
    
    # Free road term
    free_term = (v / max(v0, 0.1)) ** delta
    
    # Interaction term
    if v_lead is None or np.isinf(s_gap):
        interaction = 0.0
    else:
        dv = v - v_lead
        s_star = s0 + v * T + (v * dv) / (2.0 * np.sqrt(amax * b) + 1e-6)
        s_star = max(s0, s_star)
        interaction = (s_star / s_gap) ** 2
    
    a = amax * (1.0 - free_term - interaction)
    return a


def behavior_desired_speed(behavior: str) -> float:
    """Map behavior state to desired speed."""
    if behavior == 'CRUISE':
        return 22.0
    elif behavior == 'FOLLOW':
        return 18.0
    elif behavior == 'STOP':
        return 0.0
    elif behavior == 'ACCELERATE':
        return 28.0
    elif behavior in ['LANECHANGELEFT', 'LANECHANGERIGHT']:
        return 20.0
    else:
        return 20.0
