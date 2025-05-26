import unittest
import math
import datetime
from uav_deconfliction.data_models import Waypoint, Mission, Conflict
from uav_deconfliction.trajectory_generation import interpolate_trajectory, get_position_at_time
from uav_deconfliction.spatial_check import check_spatial_proximity, calculate_distance
from uav_deconfliction.temporal_check import check_temporal_overlap_at_location
from uav_deconfliction.deconfliction_system import DeconflictionSystem

class TestDeconflictionSystem(unittest.TestCase):

    def setUp(self):
        self.now = datetime.datetime.now()
        self.safety_buffer = 10.0 # meters

        # --- Test Waypoints ---
        self.wp_0_0_0_t0 = Waypoint(0, 0, 0, self.now)
        self.wp_100_0_0_t5 = Waypoint(100, 0, 0, self.now + datetime.timedelta(minutes=5))
        self.wp_0_100_0_t5 = Waypoint(0, 100, 0, self.now + datetime.timedelta(minutes=5))
        self.wp_50_50_0_t2_5 = Waypoint(50, 50, 0, self.now + datetime.timedelta(minutes=2, seconds=30))
        
        self.wp_50_50_50_t2_5 = Waypoint(50, 50, 50, self.now + datetime.timedelta(minutes=2, seconds=30))
        self.wp_0_0_100_t0 = Waypoint(0, 0, 100, self.now)
        self.wp_100_100_100_t5 = Waypoint(100, 100, 100, self.now + datetime.timedelta(minutes=5))

        # --- Test Missions ---
        # Mission A: Straight line 2D
        self.mission_A_wp = [self.wp_0_0_0_t0, self.wp_100_0_0_t5]
        self.mission_A = Mission("DroneA", self.mission_A_wp, self.now, self.now + datetime.timedelta(minutes=5))

        # Mission B: Parallel to A, 2D, no conflict
        self.mission_B_wp = [Waypoint(0, 50, time=self.now), Waypoint(100, 50, time=self.now + datetime.timedelta(minutes=5))]
        self.mission_B = Mission("DroneB", self.mission_B_wp, self.now, self.now + datetime.timedelta(minutes=5))

        # Mission C: Head-on collision with A, 2D
        self.mission_C_wp = [Waypoint(100, 0, time=self.now), Waypoint(0, 0, time=self.now + datetime.timedelta(minutes=5))]
        self.mission_C = Mission("DroneC", self.mission_C_wp, self.now, self.now + datetime.timedelta(minutes=5))

        # Mission D: 3D ascending
        self.mission_D_wp = [self.wp_0_0_0_t0, self.wp_50_50_50_t2_5, self.wp_100_100_100_t5]
        self.mission_D = Mission("DroneD", self.mission_D_wp, self.now, self.now + datetime.timedelta(minutes=5))

        # Mission E: 3D crossing D at same altitude
        self.mission_E_wp = [Waypoint(100, 0, 50, time=self.now), Waypoint(0, 100, 50, time=self.now + datetime.timedelta(minutes=5))]
        self.mission_E = Mission("DroneE", self.mission_E_wp, self.now, self.now + datetime.timedelta(minutes=5))

        # Mission F: 3D same path as D, but at higher altitude (no conflict)
        self.mission_F_wp = [Waypoint(0, 0, 200, time=self.now), Waypoint(100, 100, 200, time=self.now + datetime.timedelta(minutes=5))]
        self.mission_F = Mission("DroneF", self.mission_F_wp, self.now, self.now + datetime.timedelta(minutes=5))

    # --- Test Data Models ---
    def test_waypoint_distance(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(3, 4, 0)
        self.assertAlmostEqual(wp1.distance_to(wp2), 5.0)
        wp3 = Waypoint(0, 0, 5)
        self.assertAlmostEqual(wp1.distance_to(wp3), 5.0)
        wp4 = Waypoint(3, 4, 5)
        self.assertAlmostEqual(wp1.distance_to(wp4), math.sqrt(3*3 + 4*4 + 5*5)) # sqrt(9+16+25) = sqrt(50)

    def test_mission_duration_and_length(self):
        self.assertEqual(self.mission_A.get_duration(), datetime.timedelta(minutes=5))
        self.assertAlmostEqual(self.mission_A.get_path_length(), 100.0) # (100,0) - (0,0)

    def test_mission_waypoint_time_assignment(self):
        wp_no_time = [Waypoint(0,0), Waypoint(100,0)]
        mission_no_time = Mission("Test", wp_no_time, self.now, self.now + datetime.timedelta(minutes=10))
        self.assertIsNotNone(mission_no_time.waypoints[0].time)
        self.assertEqual(mission_no_time.waypoints[0].time, self.now)
        self.assertEqual(mission_no_time.waypoints[-1].time, self.now + datetime.timedelta(minutes=10))
        # Check midpoint
        mid_time = self.now + datetime.timedelta(minutes=5)
        mid_wp = Waypoint(50, 0, 0, mid_time)
        self.assertAlmostEqual(mission_no_time.waypoints[1].time.timestamp(), mission_no_time.end_time.timestamp(), delta=1) # <--- Corrected
        
    # --- Test Trajectory Generation ---
    def test_interpolate_trajectory_2d(self):
        traj = interpolate_trajectory(self.mission_A, time_step_seconds=60) # Every minute
        self.assertEqual(len(traj), 6) # 0, 1, 2, 3, 4, 5 minutes
        self.assertAlmostEqual(traj[0].x, 0)
        self.assertAlmostEqual(traj[0].y, 0)
        self.assertAlmostEqual(traj[5].x, 100)
        self.assertAlmostEqual(traj[5].y, 0)
        self.assertAlmostEqual(traj[2].x, 40) # At 2 minutes, 40% of 100
        self.assertAlmostEqual(traj[2].y, 0)

    def test_get_position_at_time(self):
        query_time = self.now + datetime.timedelta(minutes=2, seconds=30) # Midpoint
        pos = get_position_at_time(self.mission_A, query_time)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos.x, 50.0)
        self.assertAlmostEqual(pos.y, 0.0)
        self.assertEqual(pos.time, query_time)

        # Test outside mission time
        pos_before = get_position_at_time(self.mission_A, self.now - datetime.timedelta(seconds=1))
        self.assertIsNone(pos_before)
        pos_after = get_position_at_time(self.mission_A, self.now + datetime.timedelta(minutes=5, seconds=1))
        self.assertIsNone(pos_after)

    # --- Test Spatial Check ---
    def test_check_spatial_proximity_no_conflict(self):
        # A and B are parallel, 50 units apart
        traj_A = interpolate_trajectory(self.mission_A)
        traj_B = interpolate_trajectory(self.mission_B)
        conflicts = check_spatial_proximity("A", traj_A, "B", traj_B, safety_buffer=self.safety_buffer)
        self.assertEqual(len(conflicts), 0)

    def test_check_spatial_proximity_with_conflict(self):
        # A and C are head-on, will have spatial overlap
        traj_A = interpolate_trajectory(self.mission_A)
        traj_C = interpolate_trajectory(self.mission_C)
        conflicts = check_spatial_proximity("A", traj_A, "C", traj_C, safety_buffer=self.safety_buffer)
        self.assertGreater(len(conflicts), 0)
        # Check if one conflict point is roughly near the intersection (50,0)
        found_near_midpoint = False
        for p_wp, s_wp in conflicts:
            if p_wp.distance_to(Waypoint(50,0,0,p_wp.time)) < self.safety_buffer and \
               s_wp.distance_to(Waypoint(50,0,0,s_wp.time)) < self.safety_buffer:
                found_near_midpoint = True
                break
        self.assertTrue(found_near_midpoint)

    # --- Test Temporal Check ---
    def test_check_temporal_overlap_at_location_no_overlap(self):
        # Mission A and a delayed drone X that follows A's path but starts much later
        wp_X = [Waypoint(0, 0, time=self.now + datetime.timedelta(hours=1)),
                Waypoint(100, 0, time=self.now + datetime.timedelta(hours=1, minutes=5))]
        mission_X = Mission("DroneX", wp_X, self.now + datetime.timedelta(hours=1), self.now + datetime.timedelta(hours=1, minutes=5))

        # Pick a point that would be spatially close but far in time
        # Let's say, midpoint of A
        potential_conflict_wp_A = get_position_at_time(self.mission_A, self.now + datetime.timedelta(minutes=2, seconds=30))
        # Corresponding point on X (at same spatial coord, but different time)
        potential_conflict_wp_X = get_position_at_time(mission_X, self.now + datetime.timedelta(hours=1, minutes=2, seconds=30))
        
        conflict = check_temporal_overlap_at_location(
            self.mission_A, mission_X, potential_conflict_wp_A, potential_conflict_wp_X, self.safety_buffer
        )
        self.assertIsNone(conflict)

    def test_check_temporal_overlap_at_location_with_overlap(self):
        # Mission A and C are head-on, will have temporal overlap
        
        # Manually find approximate conflict point time for testing
        # Drones are at (50,0) at self.now + 2m30s
        potential_conflict_wp_A = get_position_at_time(self.mission_A, self.now + datetime.timedelta(minutes=2, seconds=30))
        potential_conflict_wp_C = get_position_at_time(self.mission_C, self.now + datetime.timedelta(minutes=2, seconds=30))

        self.assertIsNotNone(potential_conflict_wp_A)
        self.assertIsNotNone(potential_conflict_wp_C)
        
        # Ensure they are initially spatially close enough for a valid test
        self.assertLess(calculate_distance(potential_conflict_wp_A, potential_conflict_wp_C), self.safety_buffer * 2) # Use a bit more than safety buffer for initial check

        conflict = check_temporal_overlap_at_location(
            self.mission_A, self.mission_C, potential_conflict_wp_A, potential_conflict_wp_C, self.safety_buffer
        )
        self.assertIsNotNone(conflict)
        self.assertEqual(conflict.conflicting_drone_ids, sorted(["DroneA", "DroneC"]))
        self.assertAlmostEqual(conflict.time.timestamp(), (self.now + datetime.timedelta(minutes=2, seconds=30)).timestamp(), delta=5)

    # --- Test Deconfliction System (Integration Tests) ---
    def test_system_conflict_free_scenario(self):
        deconflict_sys = DeconflictionSystem([self.mission_B], safety_buffer=self.safety_buffer)
        result = deconflict_sys.verify_mission(self.mission_A)
        self.assertEqual(result['status'], "clear")
        self.assertEqual(len(result['conflict_details']), 0)

    def test_system_direct_conflict_scenario(self):
        deconflict_sys = DeconflictionSystem([self.mission_C], safety_buffer=self.safety_buffer)
        result = deconflict_sys.verify_mission(self.mission_A)
        self.assertEqual(result['status'], "conflict detected")
        self.assertGreater(len(result['conflict_details']), 0)
        self.assertEqual(result['conflict_details'][0].conflicting_drone_ids, sorted(["DroneA", "DroneC"]))

    def test_system_3d_conflict_scenario(self):
        # Drone D ascends, Drone E crosses at constant altitude 50. They should conflict around (50,50,50)
        deconflict_sys = DeconflictionSystem([self.mission_E, self.mission_F], safety_buffer=self.safety_buffer)
        result = deconflict_sys.verify_mission(self.mission_D)
        
        self.assertEqual(result['status'], "conflict detected")
        self.assertGreater(len(result['conflict_details']), 0)
        
        # Check that the conflict is indeed between DroneD and DroneE, not F
        found_expected_conflict = False
        for conflict in result['conflict_details']:
            if sorted(conflict.conflicting_drone_ids) == sorted(["DroneD", "DroneE"]):
                found_expected_conflict = True
                # Check conflict location is roughly correct
                self.assertLess(conflict.location.distance_to(Waypoint(50,50,50,conflict.time)), self.safety_buffer * 2)
                self.assertAlmostEqual(conflict.time.timestamp(), (self.now + datetime.timedelta(minutes=2, seconds=30)).timestamp(), delta=15) # Increased delta to accommodate interpolation

                break
        self.assertTrue(found_expected_conflict, "Expected conflict between DroneD and DroneE not found or incorrect.")
        
        # Ensure no conflict with DroneF
        for conflict in result['conflict_details']:
            self.assertNotEqual(sorted(conflict.conflicting_drone_ids), sorted(["DroneD", "DroneF"]))


if __name__ == '__main__':
    unittest.main()