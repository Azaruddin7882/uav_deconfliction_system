import unittest
from datetime import datetime, timedelta
from src.core.mission import DroneMission
from src.core.conflict import ConflictDetector


"""
Testing scenerios:
->Testing No Conflict (Safe Scenarios)
  To verify the system correctly identifies when drones won't collide.
    Example:
    Drone A flies left-to-right at 10 AM
    Drone B flies far away at 11 AM
    Should report no danger

-> Time Overlap But Safe Distance
   Confirms drones can fly at the same time if routes don't cross.
    Example:

    Drone A flies north path at 10 AM
    Drone B flies parallel south path at same time
    Safe despite same flight time

-> Conflict Detection (Catching Crashes)
   Ensures the system spots actual collisions.
    Example:

    Drone A crosses east-west at 10 AM
    Drone B crosses north-south through same point at same time
    Must detect this X-shaped collision

-> Same Location, Different Times
   Checks time separation prevents false alarms.
    Example:

    Both drones pass (50,50,50) but:
    Drone A at 10 AM
    Drone B at 3 PM
    Safe because of time gap

->  Vertical (Altitude) Conflicts
    Verifies 3D safety checks work up/down.
    Example:

    Drone A ascending through 90m
    Drone B descending through 90m at same time
    Must flag this elevator-style collision

-> Safety Buffer Accuracy
   Tests the 5m danger zone is respected.
    Example:

    Drones 5.1m apart → Safe (4.9m apart → Danger)
    Like testing a "no closer than 5m" forcefield

->Multiple Drone Conflicts
  Handles complex air traffic scenarios.
    Example:

    Your drone + 2 others crossing paths at different points
    Must catch all collision points

-> Invalid Mission Tests
   Catches user errors early:

    Single-waypoint missions (can't fly nowhere)
    End time before start time (time travel not allowed)
    Prevents impossible flight plans    

"""

class TestConflictDetection(unittest.TestCase):
    def setUp(self):
        self.detector = ConflictDetector(safety_buffer=5.0)
        
    def test_no_conflict(self):
        mission1 = DroneMission(
            waypoints=[[0, 0, 0], [10, 0, 0]],
            start_time=datetime(2025, 1, 1, 10, 0),
            end_time=datetime(2025, 1, 1, 10, 10)
        )
        mission2 = DroneMission(
            waypoints=[[100, 100, 100], [110, 100, 100]],
            start_time=datetime(2025, 1, 1, 11, 0),
            end_time=datetime(2025, 1, 1, 11, 10)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission2])
        self.assertEqual(len(conflicts), 0)
        
    def test_temporal_overlap_but_spatial_safe(self):
        mission1 = DroneMission(
            waypoints=[[0, 0, 0], [100, 0, 0]],
            start_time=datetime(2025, 1, 1, 10, 0),
            end_time=datetime(2025, 1, 1, 10, 10)
        )
        mission2 = DroneMission(
            waypoints=[[0, 100, 0], [100, 100, 0]],
            start_time=datetime(2025, 1, 1, 10, 5),
            end_time=datetime(2025, 1, 1, 10, 15)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission2])
        self.assertEqual(len(conflicts), 0)
        
    def test_conflict_detection(self):
        mission1 = DroneMission(
            waypoints=[[0, 0, 0], [100, 0, 0]],
            start_time=datetime(2025, 1, 1, 10, 0),
            end_time=datetime(2025, 1, 1, 10, 10)
        )
        mission2 = DroneMission(
            waypoints=[[50, -5, 0], [50, 5, 0]],
            start_time=datetime(2025, 1, 1, 10, 5),
            end_time=datetime(2025, 1, 1, 10, 6)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission2])
        self.assertGreater(len(conflicts), 0)
        
        # Verify conflict details
        conflict = conflicts[0]
        self.assertAlmostEqual(conflict['location'][0], 50, delta=2)
        self.assertAlmostEqual(conflict['location'][1], 0, delta=2)
        self.assertEqual(conflict['time'].time(), datetime(2025, 1, 1, 10, 5, 30).time())
        
    def test_edge_case_same_location_different_time(self):
        mission1 = DroneMission(
            waypoints=[[50, 50, 50], [60, 60, 60]],
            start_time=datetime(2025, 1, 1, 10, 0),
            end_time=datetime(2025, 1, 1, 10, 10)
        )
        mission2 = DroneMission(
            waypoints=[[50, 50, 50], [40, 40, 40]],
            start_time=datetime(2025, 1, 1, 10, 15),
            end_time=datetime(2025, 1, 1, 10, 25)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission2])
        self.assertEqual(len(conflicts), 0)
        
    def test_vertical_conflict(self):
        mission1 = DroneMission(
            waypoints=[[0, 0, 0], [0, 0, 100]],
            start_time=datetime(2025, 1, 1, 10, 0),
            end_time=datetime(2025, 1, 1, 10, 10)
        )
        mission2 = DroneMission(
            waypoints=[[0, 0, 90], [0, 0, 110]],
            start_time=datetime(2025, 1, 1, 10, 5),
            end_time=datetime(2025, 1, 1, 10, 15)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission2])
        self.assertGreater(len(conflicts), 0)
        
    def test_buffer_zone_respected(self):
        # These drones should pass just outside the safety buffer
        mission1 = DroneMission(
            waypoints=[[0, 0, 0], [100, 0, 0]],
            start_time=datetime(2025, 1, 1, 10, 0),
            end_time=datetime(2025, 1, 1, 10, 10)
        )
        mission2 = DroneMission(
            waypoints=[[50, 5.1, 0], [50, -5.1, 0]],  # 5.1m > 5.0m buffer
            start_time=datetime(2025, 1, 1, 10, 5),
            end_time=datetime(2025, 1, 1, 10, 6)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission2])
        self.assertEqual(len(conflicts), 0)
        
        # Now test with distance just inside buffer
        mission3 = DroneMission(
            waypoints=[[50, 4.9, 0], [50, -4.9, 0]],  # 4.9m < 5.0m buffer
            start_time=datetime(2025, 1, 1, 10, 5),
            end_time=datetime(2025, 1, 1, 10, 6)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission3])
        self.assertGreater(len(conflicts), 0)
        
    def test_multiple_conflicts(self):
        mission1 = DroneMission(
            waypoints=[[0, 0, 0], [100, 100, 100]],
            start_time=datetime(2025, 1, 1, 10, 0),
            end_time=datetime(2025, 1, 1, 10, 10)
        )
        mission2 = DroneMission(
            waypoints=[[50, 50, 50], [150, 50, 50]],
            start_time=datetime(2025, 1, 1, 10, 5),
            end_time=datetime(2025, 1, 1, 10, 15)
        )
        mission3 = DroneMission(
            waypoints=[[25, 25, 25], [25, 75, 75]],
            start_time=datetime(2025, 1, 1, 10, 2),
            end_time=datetime(2025, 1, 1, 10, 8)
        )
        conflicts = self.detector.detect_conflicts(mission1, [mission2, mission3])
        self.assertGreaterEqual(len(conflicts), 2)
        
    def test_mission_with_single_waypoint(self):
        with self.assertRaises(ValueError):
            DroneMission(
                waypoints=[[0, 0, 0]],
                start_time=datetime(2025, 1, 1, 10, 0),
                end_time=datetime(2025, 1, 1, 10, 10)
            )
            
    def test_invalid_time_window(self):
        with self.assertRaises(ValueError):
            DroneMission(
                waypoints=[[0, 0, 0], [10, 0, 0]],
                start_time=datetime(2025, 1, 1, 10, 10),
                end_time=datetime(2025, 1, 1, 10, 0)
            )

if __name__ == '__main__':
    unittest.main()