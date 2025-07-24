import numpy as np
from typing import List, Dict
from typing import List, Dict, Tuple
from datetime import datetime, timedelta
from .mission import DroneMission, Waypoint
#from ..utils.geometry import distance_4d
from utils.geometry import distance_4d

class ConflictDetector:
    def __init__(self, safety_buffer: float = 5.0, time_resolution: float = 1.0):
        self.safety_buffer = safety_buffer
        self.time_resolution = time_resolution  # seconds
        
    def detect_conflicts(self, primary: DroneMission, others: List[DroneMission]) -> List[Dict]:
        """Detect conflicts between primary mission and other drones"""
        conflicts = []
        
        # Generate trajectories for all missions
        primary.generate_trajectory(speed=10.0)  # example speed
        for other in others:
            other.generate_trajectory(speed=10.0)
            
            # Check for temporal overlap first (quick rejection)
            if not self._has_temporal_overlap(primary, other):
                continue
                
            # Detailed 4D conflict check
            conflicts.extend(self._check_4d_conflicts(primary, other))
            
        return conflicts
    
    def _has_temporal_overlap(self, mission1: DroneMission, mission2: DroneMission) -> bool:
        """Check if two missions have any time overlap"""
        return not (mission1.end_time < mission2.start_time or 
                   mission2.end_time < mission1.start_time)
    
    def _check_4d_conflicts(self, primary: DroneMission, other: DroneMission) -> List[Dict]:
        """Perform detailed 4D conflict check"""
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
        """Create time bins for efficient conflict checking"""
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
        """Get trajectory segment within a specific time range"""
        return [wp for wp in trajectory if start <= wp.timestamp <= end]