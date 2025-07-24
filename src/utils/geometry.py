import math
from datetime import datetime
from typing import Tuple

"""
Approach:

-> 3d distance calculation -> Measures straight-line distance between two points 
   in space (x,y,z coordinates)
-> 4D Distance (Space + Time) -> Combines physical distance AND time difference into one "danger score".
-> Time Weighting (Custom Danger Zone)
-> time_weight lets you decide how much to prioritize time vs. space.
-> Waypoint Interpolation (GPS Prediction)-> Predicts a drone's exact position between two known points at any given time.

Example:
the below code desinged in same as example 

1. Drone 1
Route: Warehouse → Customer's Home

Waypoints:
Start: (0, 0, 50m) at 10:00:00 AM
End: (100, 100, 50m) at 10:10:00 AM

2.Drone 2
Route: Store → Dropoff

Waypoints:
Start: (50, 50, 50m) at 10:05:00 AM
End: (150, 50, 50m) at 10:15:00 AM

Where Both Drones Are at 10:07:00 AM --- ?

Drone1:
Interpolated position: (70, 70, 50m)
Drone2:
Interpolated position: (90, 50, 50m)

-> 3d distance
distance_3d((70,70,50), (90,50,50)) 
= sqrt((90-70)² + (50-70)² + (50-50)²) 
= sqrt(400 + 400 + 0) 
= 28.28 meters

4D Distance (With Time Weight = 1)
Both drones are at 10:07:00 AM → 0 time difference
distance_4d = sqrt(28.28² + 0²) = 28.28 units

case-1 :Near-Miss at 10:07:30 AM
Drone1: (75, 75, 50m)
Drone2: (95, 50, 50m)

3D distance: 29.15m
Time difference: 0 seconds → Still safe

case-2: Danger Zone at 10:08:00 AM

Drone1: (80, 80, 50m)
Drone2: (100, 50, 50m)

3D distance: 36.06m
No time difference → Safe

Collision: Distance = 0m at the same time.
Near-Miss: Distance < safety buffer 
Safe: Distance ≥ safety buffer.


"""

def distance_3d(p1: Tuple[float, float, float], p2: Tuple[float, float, float]) -> float:
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def distance_4d(p1, p2, time_weight: float = 1.0) -> float:
    spatial_dist = distance_3d((p1.x, p1.y, p1.z), (p2.x, p2.y, p2.z))
    time_diff = abs((p1.timestamp - p2.timestamp).total_seconds())
    weighted_time = time_diff * time_weight
    
    # Combine spatial and temporal distances
    return math.sqrt(spatial_dist**2 + weighted_time**2)

def interpolate_waypoints(wp1: 'Waypoint', wp2: 'Waypoint', timestamp: datetime) -> 'Waypoint':
    total_time = (wp2.timestamp - wp1.timestamp).total_seconds()
    elapsed = (timestamp - wp1.timestamp).total_seconds()
    ratio = elapsed / total_time
    
    x = wp1.x + ratio * (wp2.x - wp1.x)
    y = wp1.y + ratio * (wp2.y - wp1.y)
    z = wp1.z + ratio * (wp2.z - wp1.z)
    
    return Waypoint(x, y, z, timestamp)