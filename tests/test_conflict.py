import unittest
from datetime import datetime, timedelta
from src.core.mission import DroneMission
from src.core.conflict import ConflictDetector

class TestConflictDetection(unittest.TestCase):
    def setUp(self):
        self.detector = ConflictDetector(safety_buffer=5.0)
        
    def test_no_conflict(self):
        """Test missions that don't overlap in time or space"""
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
        """Test missions that overlap in time but not in space"""
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
        """Test missions that should have a conflict"""
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
        """Test same location but different times - should be safe"""
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
        """Test conflict detection in vertical space (altitude)"""
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
        """Test that safety buffer is properly respected"""
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
        """Test detection of multiple conflicts with multiple drones"""
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
        """Test handling of mission with only one waypoint (should raise error)"""
        with self.assertRaises(ValueError):
            DroneMission(
                waypoints=[[0, 0, 0]],
                start_time=datetime(2025, 1, 1, 10, 0),
                end_time=datetime(2025, 1, 1, 10, 10)
            )
            
    def test_invalid_time_window(self):
        """Test handling of invalid time window (end before start)"""
        with self.assertRaises(ValueError):
            DroneMission(
                waypoints=[[0, 0, 0], [10, 0, 0]],
                start_time=datetime(2025, 1, 1, 10, 10),
                end_time=datetime(2025, 1, 1, 10, 0)
            )

if __name__ == '__main__':
    unittest.main()