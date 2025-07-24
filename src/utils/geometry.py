import math
from datetime import datetime
from typing import Tuple

def distance_3d(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
    """Calculate 3D Euclidean distance between two points"""
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def distance_4d(p1, p2, time_weight: float = 1.0) -> float:
    """
    Calculate 4D distance (3D space + time) between two waypoints
    time_weight: scaling factor to convert time difference to equivalent distance
    """
    spatial_dist = distance_3d((p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z))
    time_diff = abs((p1.timestamp - p2.timestamp).total_seconds())
    weighted_time = time_diff * time_weight
    
    # Combine spatial and temporal distances
    return math.sqrt(spatial_dist**2 + weighted_time**2)

def interpolate_waypoints(wp1: 'Waypoint', wp2: 'Waypoint', timestamp: datetime) -> 'Waypoint':
    """Interpolate between two waypoints at a specific timestamp"""
    total_time = (wp2.timestamp - wp1.timestamp).total_seconds()
    elapsed = (timestamp - wp1.timestamp).total_seconds()
    ratio = elapsed / total_time
    
    x = wp1.x + ratio * (wp2.x - wp1.x)
    y = wp1.y + ratio * (wp2.y - wp1.y)
    z = wp1.z + ratio * (wp2.z - wp1.z)
    
    return Waypoint(x, y, z, timestamp)