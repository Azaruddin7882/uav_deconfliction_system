import numpy as np
from typing import List, Dict
from typing import List, Dict, Tuple
from datetime import datetime, timedelta
from .mission import DroneMission, Waypoint
#from ..utils.geometry import distance_4d
from utils.geometry import distance_4d

'''
Approach:

-> The system checks if drones could come closer than 5 meters at any point during their flights.
-> It divides the flight time into 1-second chunks to check positions at regular intervals.
-> First eliminates drone pairs that won't fly at the same time 
-> For drones flying at the same time, compares their positions (x,y,z) AND timestamps to find conflicts.
-> When conflicts are found, records exactly when/where they occur and how close the drones get.

example calulation:

let us consider:

Drone P 
Flies from (0,0,0) at 10:00:00 to (10,10,10) at 10:00:10
Speed: 1 m/s

Drone O 
Flies from (5,5,5) at 10:00:05 to (15,15,15) at 10:00:15
Speed: 1 m/

Conflict Check at 10:00:07:

Drone P position: (7,7,7)

Drone O position: (7,7,7)

Distance: 0 meters (CONFLICT!) --- > collision
 
Out put looks in below format for above case:

{
  "time": "10:00:07",
  "location": (7,7,7), 
  "distance": 0.0,
  "conflicting_drone": "O"
}

other cases:
{
    "time": "2023-11-15T10:06:30",  ---> 30 seconds before collision
    "location": [6.5, 6.5, 6.5],
    "distance": 2.1,                 
    "conflicting_drone": "delivery_drone_55"
}

'''

class ConflictDetector:
    def __init__(self, safety_buffer: float = 5.0, time_resolution: float = 1.0):
        self.safety_buffer = safety_buffer
        self.time_resolution = time_resolution  
        
    def detect_conflicts(self, primary: DroneMission, others: List[DroneMission]) -> List[Dict]:
        conflicts = []
        
        # Generate trajectories for all missions
        primary.generate_trajectory(speed=10.0)  
        for other in others:
            other.generate_trajectory(speed=10.0)
            
            # Check for temporal overlap first (quick rejection)
            if not self._has_temporal_overlap(primary, other):
                continue
                
            # Detailed 4D conflict check
            conflicts.extend(self._check_4d_conflicts(primary, other))
            
        return conflicts
    
    def _has_temporal_overlap(self, mission1: DroneMission, mission2: DroneMission) -> bool:
        return not (mission1.end_time < mission2.start_time or 
                   mission2.end_time < mission1.start_time)
    
    def _check_4d_conflicts(self, primary: DroneMission, other: DroneMission) -> List[Dict]:
        conflicts = []
        
        # Create time bins for efficient comparison
        time_bins = self._create_time_bins(primary, other)
        
        for bin_start, bin_end in time_bins:
            primary_segment = self._get_segment_in_time(primary.trajectory, bin_start, bin_end)
            other_segment = self._get_segment_in_time(other.trajectory, bin_start, bin_end)
            
            # Check all point combinations in this time bin
            for p1 in primary_segment:
                for p2 in other_segment:
                    dist = distance_4d(p1, p2, self.time_resolution)
                    if dist < self.safety_buffer:
                        conflicts.append({
                            "time": p1.timestamp,
                            "location": (p1.x, p1.y, p1.z),
                            "distance": dist,
                            "conflicting_drone": other.id if hasattr(other, 'id') else "unknown"
                        })
                        
        return conflicts
    
    def _create_time_bins(self, mission1: DroneMission, mission2: DroneMission) -> List[Tuple[datetime, datetime]]:
        start = max(mission1.start_time, mission2.start_time)
        end = min(mission1.end_time, mission2.end_time)
        
        bins = []
        current = start
        while current < end:
            next_time = current + timedelta(seconds=self.time_resolution)
            bins.append((current, min(next_time, end)))
            current = next_time
            
        return bins
    
    def _get_segment_in_time(self, trajectory: List[Waypoint], start: datetime, end: datetime) -> List[Waypoint]:
        return [wp for wp in trajectory if start <= wp.timestamp <= end]