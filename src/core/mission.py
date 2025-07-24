import json
from dataclasses import dataclass
from typing import List, Tuple, Optional
from datetime import datetime, timedelta
import math

@dataclass
class Waypoint:
    x: float
    y: float
    z: float = 0.0  # Default to 2D if not specified
    timestamp: datetime = None  # Will be populated during trajectory generation

class DroneMission:
    def __init__(self, waypoints: List[Tuple[float, float, float]], 
                 start_time: datetime, end_time: datetime):
        if len(waypoints) < 2:
            raise ValueError("At least two waypoints are required for a mission")
            
        self.waypoints = [Waypoint(x, y, z) for x, y, z in waypoints]
        self.start_time = start_time
        self.end_time = end_time
        self.trajectory: List[Waypoint] = []
        self._validate_mission_time()
        
    def _validate_mission_time(self):
        """Ensure the mission has a valid time window"""
        if self.end_time <= self.start_time:
            raise ValueError("End time must be after start time")
    
    def generate_trajectory(self, speed: float, acceleration: float = 2.0, deceleration: float = 2.0):
        """
        Interpolate waypoints into a continuous trajectory with timestamps
        accounting for acceleration and deceleration between waypoints.
        
        Args:
            speed: Cruising speed in m/s
            acceleration: Acceleration rate in m/s² (default: 2.0)
            deceleration: Deceleration rate in m/s² (default: 2.0)
        """
        if speed <= 0:
            raise ValueError("Speed must be positive")
        if acceleration <= 0 or deceleration <= 0:
            raise ValueError("Acceleration and deceleration must be positive")
            
        self.trajectory = []
        current_time = self.start_time
        
        # Add the first waypoint with start time
        self.waypoints[0].timestamp = current_time
        self.trajectory.append(self.waypoints[0])
        
        for i in range(len(self.waypoints) - 1):
            wp1 = self.waypoints[i]
            wp2 = self.waypoints[i + 1]
            
            # Calculate distance between waypoints
            distance = math.sqrt(
                (wp2.x - wp1.x)**2 + 
                (wp2.y - wp1.y)**2 + 
                (wp2.z - wp1.z)**2
            )
            
            if distance == 0:  # Same waypoint
                wp2.timestamp = current_time
                self.trajectory.append(wp2)
                continue
                
            # Calculate time needed for this segment
            # Time to accelerate to cruising speed
            t_accel = speed / acceleration
            dist_accel = 0.5 * acceleration * t_accel**2
            
            # Time to decelerate from cruising speed
            t_decel = speed / deceleration
            dist_decel = 0.5 * deceleration * t_decel**2
            
            if dist_accel + dist_decel > distance:
                # Not enough distance to reach full speed - triangular profile
                max_reachable_speed = math.sqrt(
                    (2 * acceleration * deceleration * distance) / 
                    (acceleration + deceleration)
                )
                segment_time = timedelta(
                    seconds=max_reachable_speed / acceleration + 
                           max_reachable_speed / deceleration
                )
                
                # Generate points for triangular profile
                num_points = max(2, int(distance))
                for j in range(1, num_points + 1):
                    ratio = j / num_points
                    t = ratio * segment_time.total_seconds()
                    
                    if t <= t_accel:
                        # Acceleration phase
                        current_speed = acceleration * t
                        dist_covered = 0.5 * acceleration * t**2
                    else:
                        # Deceleration phase
                        current_speed = max_reachable_speed - deceleration * (t - t_accel)
                        dist_covered = dist_accel + max_reachable_speed * (t - t_accel) - 0.5 * deceleration * (t - t_accel)**2
                    
                    ratio_covered = dist_covered / distance
                    x = wp1.x + ratio_covered * (wp2.x - wp1.x)
                    y = wp1.y + ratio_covered * (wp2.y - wp1.y)
                    z = wp1.z + ratio_covered * (wp2.z - wp1.z)
                    point_time = current_time + timedelta(seconds=t)
                    
                    self.trajectory.append(Waypoint(x, y, z, point_time))
            else:
                # Trapezoidal speed profile - accelerate, cruise, decelerate
                cruise_dist = distance - dist_accel - dist_decel
                cruise_time = cruise_dist / speed
                segment_time = timedelta(
                    seconds=t_accel + cruise_time + t_decel
                )
                
                # Generate points for trapezoidal profile
                num_points = max(2, int(distance))
                for j in range(1, num_points + 1):
                    ratio = j / num_points
                    dist_covered = ratio * distance
                    
                    if dist_covered <= dist_accel:
                        # Acceleration phase
                        t = math.sqrt(2 * dist_covered / acceleration)
                    elif dist_covered <= dist_accel + cruise_dist:
                        # Cruise phase
                        t = t_accel + (dist_covered - dist_accel) / speed
                    else:
                        # Deceleration phase
                        remaining_dist = distance - dist_covered
                        t = segment_time.total_seconds() - math.sqrt(2 * remaining_dist / deceleration)
                    
                    ratio_covered = dist_covered / distance
                    x = wp1.x + ratio_covered * (wp2.x - wp1.x)
                    y = wp1.y + ratio_covered * (wp2.y - wp1.y)
                    z = wp1.z + ratio_covered * (wp2.z - wp1.z)
                    point_time = current_time + timedelta(seconds=t)
                    
                    self.trajectory.append(Waypoint(x, y, z, point_time))
            
            current_time += segment_time
            wp2.timestamp = current_time
            self.trajectory.append(wp2)

    def get_position_at_time(self, time: datetime) -> Optional[Waypoint]:
        """
        Get the drone's position at a specific time by interpolating the trajectory.
        Returns None if the time is outside the mission window.
        """
        if time < self.start_time or time > self.end_time:
            return None
            
        if not self.trajectory:
            self.generate_trajectory(speed=10.0)  # Default speed if not generated
            
        # Binary search to find the segment containing the requested time
        low = 0
        high = len(self.trajectory) - 1
        
        while low <= high:
            mid = (low + high) // 2
            if self.trajectory[mid].timestamp < time:
                low = mid + 1
            elif self.trajectory[mid].timestamp > time:
                high = mid - 1
            else:
                return self.trajectory[mid]
        
        # Interpolate between the found points
        if high < 0:
            return self.trajectory[0]
        if low >= len(self.trajectory):
            return self.trajectory[-1]
            
        wp1 = self.trajectory[high]
        wp2 = self.trajectory[low]
        
        if wp1.timestamp == wp2.timestamp:
            return wp1
            
        ratio = ((time - wp1.timestamp).total_seconds() / 
                (wp2.timestamp - wp1.timestamp).total_seconds())
        
        x = wp1.x + ratio * (wp2.x - wp1.x)
        y = wp1.y + ratio * (wp2.y - wp1.y)
        z = wp1.z + ratio * (wp2.z - wp1.z)
        
        return Waypoint(x, y, z, time)

    def to_dict(self):
        return {
            "waypoints": [(wp.x, wp.y, wp.z) for wp in self.waypoints],
            "start_time": self.start_time.isoformat(),
            "end_time": self.end_time.isoformat(),
            "trajectory": [
                {"x": wp.x, "y": wp.y, "z": wp.z, "timestamp": wp.timestamp.isoformat()}
                for wp in self.trajectory
            ] if self.trajectory else []
        }

class MissionLoader:
    @staticmethod
    def load_primary_mission(file_path: str) -> DroneMission:
        """Load primary mission from JSON file"""
        with open(file_path) as f:
            data = json.load(f)
        return DroneMission(
            waypoints=data["waypoints"],
            start_time=datetime.fromisoformat(data["start_time"]),
            end_time=datetime.fromisoformat(data["end_time"])
        )
    
    @staticmethod
    def load_simulated_flights(file_path: str) -> List[DroneMission]:
        """Load simulated flights from JSON file"""
        with open(file_path) as f:
            data = json.load(f)
        return [DroneMission(
            waypoints=flight["waypoints"],
            start_time=datetime.fromisoformat(flight["start_time"]),
            end_time=datetime.fromisoformat(flight["end_time"])
        ) for flight in data["flights"]]