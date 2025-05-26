import unittest
import datetime
from uav_deconfliction.data_models import Waypoint, Mission, Conflict
from uav_deconfliction.trajectory_generation import get_position_at_time
from uav_deconfliction.temporal_check import check_temporal_overlap_at_location
from uav_deconfliction.spatial_check import calculate_distance # For setting up test conditions

class TestTemporalCheck(unittest.TestCase):

    def setUp(self):
        self.now = datetime.datetime.now().replace(microsecond=0)
        self.safety_buffer = 10.0 # meters
        self.time_proximity_threshold = datetime.timedelta(seconds=5)

        # --- Missions for testing ---
        # Mission A: (0,0) to (100,0) in 10 minutes
        self.wp_A1 = Waypoint(0, 0, 0, self.now)
        self.wp_A2 = Waypoint(100, 0, 0, self.now + datetime.timedelta(minutes=10))
        self.mission_A = Mission("DroneA", [self.wp_A1, self.wp_A2], 
                                 self.now, self.now + datetime.timedelta(minutes=10))

        # Mission B: (100,0) to (0,0) in 10 minutes (Head-on with A, concurrent)
        self.wp_B1 = Waypoint(100, 0, 0, self.now)
        self.wp_B2 = Waypoint(0, 0, 0, self.now + datetime.timedelta(minutes=10))
        self.mission_B = Mission("DroneB", [self.wp_B1, self.wp_B2], 
                                 self.now, self.now + datetime.timedelta(minutes=10))
        # Expected collision at (50,0,0) at self.now + 5 minutes

        # Mission C: (100,0) to (0,0) but starts much later (No temporal overlap with A)
        self.mission_C_start_time = self.now + datetime.timedelta(minutes=20)
        self.wp_C1 = Waypoint(100, 0, 0, self.mission_C_start_time)
        self.wp_C2 = Waypoint(0, 0, 0, self.mission_C_start_time + datetime.timedelta(minutes=10))
        self.mission_C = Mission("DroneC", [self.wp_C1, self.wp_C2], 
                                 self.mission_C_start_time, self.mission_C_start_time + datetime.timedelta(minutes=10))

        # Mission D: Crosses A's path (50,0) but at higher Z (No 2D conflict)
        self.wp_D1 = Waypoint(50, -50, 50, self.now + datetime.timedelta(minutes=5))
        self.wp_D2 = Waypoint(50, 50, 50, self.now + datetime.timedelta(minutes=5)) # Static at (50,0,50) for a moment to test
        self.mission_D = Mission("DroneD", [Waypoint(50,-50,50,self.now+datetime.timedelta(minutes=4)),
                                            Waypoint(50,50,50,self.now+datetime.timedelta(minutes=6))],
                                  self.now+datetime.timedelta(minutes=4), self.now+datetime.timedelta(minutes=6))
        
        # Mission E: A drone that barely avoids conflict in 3D
        self.wp_E1 = Waypoint(0,0,11, self.now) # Just above safety buffer
        self.wp_E2 = Waypoint(100,0,11, self.now + datetime.timedelta(minutes=10))
        self.mission_E = Mission("DroneE", [self.wp_E1, self.wp_E2], self.now, self.now + datetime.timedelta(minutes=10))


    def test_check_temporal_overlap_at_location_true_conflict_2d(self):
        # Drones A and B are head-on, will meet at (50,0) at 5 minutes
        conflict_time = self.now + datetime.timedelta(minutes=5)
        
        # Get potential conflict points near the expected meeting point
        # These would come from check_spatial_proximity
        pos_A_at_conflict = get_position_at_time(self.mission_A, conflict_time)
        pos_B_at_conflict = get_position_at_time(self.mission_B, conflict_time)
        
        self.assertIsNotNone(pos_A_at_conflict)
        self.assertIsNotNone(pos_B_at_conflict)
        self.assertLess(calculate_distance(pos_A_at_conflict, pos_B_at_conflict), self.safety_buffer)

        conflict = check_temporal_overlap_at_location(
            self.mission_A, self.mission_B, pos_A_at_conflict, pos_B_at_conflict,
            self.safety_buffer, self.time_proximity_threshold
        )
        
        self.assertIsNotNone(conflict)
        self.assertEqual(conflict.conflicting_drone_ids, ["DroneA", "DroneB"])
        self.assertAlmostEqual(conflict.location.x, 50, delta=0.1) # Midpoint
        self.assertAlmostEqual(conflict.location.y, 0, delta=0.1)
        self.assertAlmostEqual(conflict.time.timestamp(), conflict_time.timestamp(), delta=self.time_proximity_threshold.total_seconds())

    def test_check_temporal_overlap_at_location_no_temporal_overlap(self):
        # Mission A and C are spatially overlapping (same line), but C starts much later
        
        # Get a spatial overlap point for A (e.g., at 5 min for A)
        pos_A_at_5min = get_position_at_time(self.mission_A, self.now + datetime.timedelta(minutes=5)) # (50,0,0)
        
        # Get a spatial overlap point for C (at 5 min for C)
        # Note: This is 5 min into C's mission, which is self.mission_C_start_time + 5min
        pos_C_at_5min_C_mission = get_position_at_time(self.mission_C, self.mission_C_start_time + datetime.timedelta(minutes=5)) # (50,0,0)
        
        # Manually create a potential conflict pair. Their 'time' attributes will be far apart.
        potential_conflict_wp_A = Waypoint(50, 0, 0, self.now + datetime.timedelta(minutes=5))
        potential_conflict_wp_C = Waypoint(50, 0, 0, self.mission_C_start_time + datetime.timedelta(minutes=5))

        # They are spatially at (50,0,0), but time is far apart (5 mins vs 25 mins)
        self.assertGreater(abs((potential_conflict_wp_A.time - potential_conflict_wp_C.time).total_seconds()), self.time_proximity_threshold.total_seconds() * 2)

        conflict = check_temporal_overlap_at_location(
            self.mission_A, self.mission_C, potential_conflict_wp_A, potential_conflict_wp_C,
            self.safety_buffer, self.time_proximity_threshold
        )
        
        self.assertIsNone(conflict) # Should be None because times are too far apart

    def test_check_temporal_overlap_at_location_no_spatial_overlap_at_time(self):
        # Mission A and D. D is always at Z=50, A is at Z=0. No conflict
        conflict_time = self.now + datetime.timedelta(minutes=5)
        
        pos_A_at_5min = get_position_at_time(self.mission_A, conflict_time) # (50,0,0)
        pos_D_at_5min = get_position_at_time(self.mission_D, conflict_time) # (50,0,50)
        
        self.assertIsNotNone(pos_A_at_5min)
        self.assertIsNotNone(pos_D_at_5min)
        
        # Their distance is 50m, which is > safety_buffer (10m)
        self.assertGreater(calculate_distance(pos_A_at_5min, pos_D_at_5min), self.safety_buffer)

        conflict = check_temporal_overlap_at_location(
            self.mission_A, self.mission_D, pos_A_at_5min, pos_D_at_5min,
            self.safety_buffer, self.time_proximity_threshold
        )
        self.assertIsNone(conflict) # No conflict because they are not spatially close enough

    def test_check_temporal_overlap_at_location_3d_no_conflict_by_buffer(self):
        # Mission A and E. E is at Z=11, A is at Z=0. Distance is 11m. Safety buffer is 10m.
        # So they should not conflict.
        conflict_time = self.now + datetime.timedelta(minutes=5)
        
        pos_A_at_5min = get_position_at_time(self.mission_A, conflict_time) # (50,0,0)
        pos_E_at_5min = get_position_at_time(self.mission_E, conflict_time) # (50,0,11)

        self.assertIsNotNone(pos_A_at_5min)
        self.assertIsNotNone(pos_E_at_5min)
        
        # Distance is exactly 11, which is > 10 (safety_buffer)
        self.assertAlmostEqual(calculate_distance(pos_A_at_5min, pos_E_at_5min), 11.0)

        conflict = check_temporal_overlap_at_location(
            self.mission_A, self.mission_E, pos_A_at_5min, pos_E_at_5min,
            self.safety_buffer, self.time_proximity_threshold
        )
        self.assertIsNone(conflict) # No conflict because they are just outside safety buffer

    def test_check_temporal_overlap_at_location_invalid_inputs(self):
        # Test invalid mission inputs
        with self.assertRaises(TypeError):
            check_temporal_overlap_at_location("not_mission", self.mission_B, self.wp_A1, self.wp_B1, self.safety_buffer)
        
        # Test invalid waypoint inputs
        with self.assertRaises(TypeError):
            check_temporal_overlap_at_location(self.mission_A, self.mission_B, "not_waypoint", self.wp_B1, self.safety_buffer)

        # Test invalid safety_buffer
        with self.assertRaises(ValueError):
            check_temporal_overlap_at_location(self.mission_A, self.mission_B, self.wp_A1, self.wp_B1, 0)
        
        # Test invalid time_proximity_threshold
        with self.assertRaises(ValueError):
            check_temporal_overlap_at_location(self.mission_A, self.mission_B, self.wp_A1, self.wp_B1, self.safety_buffer, datetime.timedelta(seconds=-1))


if __name__ == '__main__':
    unittest.main()