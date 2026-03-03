"""
Lanelet Map Representation and A* Global Routing

This module handles:
- Lanelet-based highway map representation
- Lanelet graph construction for routing
- A* search for optimal lane sequence
"""

import numpy as np
import networkx as nx
from typing import List, Tuple, Dict, Optional


# Global constants
LANEWIDTH = 3.7  # lane width in meters


def lanecentery(laneindex):
    """
    Calculate center y-coordinate for a given lane.
    Lane 1 center is y=0, lane 0 is negative, lane 2 is positive.
    """
    return (laneindex - 1) * LANEWIDTH


class LaneletMap:
    """
    Straight highway lanelet map.
    Nodes are (laneindex, segmentindex).
    """
    
    def __init__(self, length=500.0, segmentlength=10.0, nlanes=3):
        """
        Initialize highway lanelet map.
        
        Args:
            length: Total highway length in meters
            segmentlength: Length of each lanelet segment
            nlanes: Number of parallel lanes
        """
        self.length = length
        self.segmentlength = segmentlength
        self.nlanes = nlanes
        self.nsegments = int(length / segmentlength)
        self.segmentcenters = np.arange(self.nsegments) * segmentlength + segmentlength / 2.0
        
        # Build graph
        self.G = nx.DiGraph()
        
        # Add nodes
        for lane in range(nlanes):
            for seg in range(self.nsegments):
                self.G.add_node((lane, seg))
        
        # Add edges
        for lane in range(nlanes):
            for seg in range(self.nsegments):
                # Same-lane forward edges
                if seg < self.nsegments - 1:
                    self.G.add_edge((lane, seg), (lane, seg+1), cost=1.0)
                
                # Lane change edges
                if lane > 0:
                    if seg < self.nsegments - 1:
                        self.G.add_edge((lane, seg), (lane-1, seg+1), cost=1.5)
                if lane < nlanes - 1:
                    if seg < self.nsegments - 1:
                        self.G.add_edge((lane, seg), (lane+1, seg+1), cost=1.5)
    
    def nearestnode(self, lane, s):
        """Find nearest node to given (lane, s) position."""
        segidx = int(np.clip(s // self.segmentlength, 0, self.nsegments - 1))
        return (lane, segidx)
    
    def nodetoxy(self, node):
        """Convert node to world (x, y) coordinates."""
        lane, seg = node
        x = self.segmentcenters[seg]
        y = lanecentery(lane)
        return x, y


def laneletheuristic(n1, n2):
    """
    Heuristic for A*: estimates cost between two lanelet nodes.
    
    Args:
        n1: Tuple (lane1, seg1)
        n2: Tuple (lane2, seg2)
    
    Returns:
        Estimated cost (float)
    """
    lane1, seg1 = n1
    lane2, seg2 = n2
    return abs(seg2 - seg1) + 0.5 * abs(lane2 - lane1)


def laneletastar(lmap: LaneletMap, startnode, goalnode):
    """
    A* algorithm for finding optimal lanelet sequence.
    
    Args:
        lmap: LaneletMap object
        startnode: Start node tuple (lane, seg)
        goalnode: Goal node tuple (lane, seg)
    
    Returns:
        List of nodes representing path, or None if no path found
    """
    G = lmap.G
    import heapq
    
    openset = []
    heapq.heappush(openset, (0.0, startnode))
    
    camefrom = {}
    gscore = {startnode: 0.0}
    
    while openset:
        _, current = heapq.heappop(openset)
        
        if current == goalnode:
            # Reconstruct path
            path = []
            while current in camefrom:
                path.append(current)
                current = camefrom[current]
            path.append(startnode)
            return path[::-1]
        
        for neighbor in G.successors(current):
            cost = G.edges[current, neighbor]['cost']
            tentative_g = gscore[current] + cost
            
            if tentative_g < gscore.get(neighbor, float('inf')):
                gscore[neighbor] = tentative_g
                camefrom[neighbor] = current
                f = tentative_g + laneletheuristic(neighbor, goalnode)
                heapq.heappush(openset, (f, neighbor))
    
    return None


def buildroutelaneprofile(lmap: LaneletMap, globalpathnodes):
    """
    Create desired lane function from lanelet A* path.
    
    Args:
        lmap: LaneletMap object
        globalpathnodes: List of nodes from A* path
    
    Returns:
        Tuple: (desiredlanefn, sarray, lanearray)
    """
    if not globalpathnodes:
        return None, None, None
    
    # Build segment-to-lane mapping
    segtolane = {}
    for lane, seg in globalpathnodes:
        segtolane[seg] = lane
    
    # Create arrays
    slist = []
    lanelist = []
    for seg, lane in segtolane.items():
        scenter = lmap.segmentcenters[seg]
        slist.append(scenter)
        lanelist.append(lane)
    
    sarray = np.array(slist)
    lanearray = np.array(lanelist)
    
    def desiredlanefn(s):
        segidx = int(np.clip(s // lmap.segmentlength, 0, lmap.nsegments - 1))
        idx = np.argmin(np.abs(lmap.segmentcenters - lmap.segmentcenters[segidx]))
        return int(lanearray[np.clip(idx, 0, len(lanearray)-1)])
    
    return desiredlanefn, sarray, lanearray
