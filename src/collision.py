"""
Collision Detection using Separating Axis Theorem (SAT)

This module handles:
- Vehicle rectangle representation
- SAT-based collision checking
- No false positives (unlike AABB)
"""

import numpy as np
from typing import Tuple

VEHICLELENGTH = 4.5
VEHICLEWIDTH = 2.0


def get_vehicle_corners(v) -> np.ndarray:
    """
    Return 4 corners of oriented rectangle in world coords (x,y).
    
    Args:
        v: Vehicle object with x, y, yaw attributes
    
    Returns:
        4x2 array of corner coordinates
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


def project_polygon(axis: np.ndarray, corners: np.ndarray) -> Tuple[float, float]:
    """
    Project polygon onto an axis using dot product.
    
    Args:
        axis: Unit vector defining the projection axis
        corners: Nx2 array of polygon vertices
    
    Returns:
        Tuple: (min_projection, max_projection)
    """
    dots = corners @ axis
    return np.min(dots), np.max(dots)


def rects_intersect(v1, v2) -> bool:
    """
    Check if two vehicles (axis-aligned or rotated rectangles) collide
    using the Separating Axis Theorem (SAT).
    
    SAT: Two convex polygons do NOT intersect if and only if there exists
    a line (axis) such that the projections of the two polygons do not overlap.
    
    For rectangles, we only need to check the normals to each edge.
    
    Args:
        v1, v2: Vehicle objects
    
    Returns:
        True if collision detected, False otherwise
    """
    c1 = get_vehicle_corners(v1)
    c2 = get_vehicle_corners(v2)
    
    # Axes to test (normals to edges of both rectangles)
    axes = []
    
    # Axes from v1 edges
    for i in range(4):
        edge = c1[(i+1) % 4] - c1[i]
        n = np.array([-edge[1], edge[0]])  # Perpendicular to edge
        
        if np.linalg.norm(n) < 1e-6:
            continue
        
        n = n / np.linalg.norm(n)
        axes.append(n)
    
    # Axes from v2 edges
    for i in range(4):
        edge = c2[(i+1) % 4] - c2[i]
        n = np.array([-edge[1], edge[0]])
        
        if np.linalg.norm(n) < 1e-6:
            continue
        
        n = n / np.linalg.norm(n)
        axes.append(n)
    
    # Check for separating axis
    for axis in axes:
        min1, max1 = project_polygon(axis, c1)
        min2, max2 = project_polygon(axis, c2)
        
        # If projections don't overlap, we found a separating axis
        if max1 < min2 or max2 < min1:
            return False  # No collision
    
    # No separating axis found -> collision detected
    return True
