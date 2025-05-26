import unittest
import datetime
import math
from uav_deconfliction.data_models import Waypoint, Mission
from uav_deconfliction.trajectory_generation import interpolate_trajectory
from uav_deconfliction.spatial_check import calculate_distance, check_spatial_proximity

class TestSpatialCheck(unittest.TestCase):

    def setUp(self):
        self.now = datetime.datetime.now().replace(microsecond=0)
        self.safety_buffer = 10.0 # meters

        # --- Missions for testing ---
        # Mission A: 2D, (0,0) to (100,0) in 5 min
        self.mission_A_wp = [
            Waypoint(0, 0, time=self.now),
            Waypoint(100, 0, time=self.now + datetime.timedelta(minutes=5))
        ]
        self.mission_A = Mission("DroneA", self.mission_A_wp, self.now, self.now + datetime.timedelta(minutes=5))
        self.traj_A = interpolate_trajectory(self.mission_A, time_step_seconds=1)

        # Mission B: 2D, (0,15) to (100,15) in 5 min (15m parallel, no conflict with 10m buffer)
        self.mission_B_wp = [
            Waypoint(0, 15, time=self.now),
            Waypoint(100, 15, time=self.now + datetime.timedelta(minutes=5))
        ]
        self.mission_B = Mission("DroneB", self.mission_B_wp, self.now, self.now + datetime.timedelta(minutes=5))
        self.traj_B = interpolate_trajectory(self.mission_B, time_step_seconds=1)

        # Mission C: 2D, (0,5) to (100,5) in 5 min (5m parallel, CONFLICT with 10m buffer)
        self.mission_C_wp = [
            Waypoint(0, 5, time=self.now),
            Waypoint(100, 5, time=self.now + datetime.timedelta(minutes=5))
        ]
        self.mission_C = Mission("DroneC", self.mission_C_wp, self.now, self.now + datetime.timedelta(minutes=5))
        self.traj_C = interpolate_trajectory(self.mission_C, time_step_seconds=1)

        # Mission D: 2D, (100,0) to (0,0) in 5 min (Head-on with A, CONFLICT)
        self.mission_D_wp = [
            Waypoint(100, 0, time=self.now),
            Waypoint(0, 0, time=self.now + datetime.timedelta(minutes=5))
        ]
        self.mission_D = Mission("DroneD", self.mission_D_wp, self.now, self.now + datetime.timedelta(minutes=5))
        self.traj_D = interpolate_trajectory(self.mission_D, time_step_seconds=1)

        # Mission E: 3D, (0,0,0) to (100,0,0) in 5 min
        self.mission_E_wp = [
            Waypoint(0, 0, 0, time=self.now),
            Waypoint(100, 0, 0, time=self.now + datetime.timedelta(minutes=5))
        ]
        self.mission_E = Mission("DroneE", self.mission_E_wp, self.now, self.now + datetime.timedelta(minutes=5))
        self.traj_E = interpolate_trajectory(self.mission_E, time_step_seconds=1)

        # Mission F: 3D, (0,0,5) to (100,0,5) in 5 min (5m above E, CONFLICT with 10m buffer)
        self.mission_F_wp = [
            Waypoint(0, 0, 5, time=self.now),
            Waypoint(100, 0, 5, time=self.now + datetime.timedelta(minutes=5))
        ]
        self.mission_F = Mission("DroneF", self.mission_F_wp, self.now, self.now + datetime.timedelta(minutes=5))
        self.traj_F = interpolate_trajectory(self.mission_F, time_step_seconds=1)


    # --- Test calculate_distance (re-test for module scope) ---
    def test_calculate_distance_2d(self):
        wp1 = Waypoint(0, 0)
        wp2 = Waypoint(3, 4)
        self.assertAlmostEqual(calculate_distance(wp1, wp2), 5.0)

    def test_calculate_distance_3d(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(3, 4, 5)
        self.assertAlmostEqual(calculate_distance(wp1, wp2), math.sqrt(50))

    # --- Test check_spatial_proximity ---
    def test_check_spatial_proximity_no_overlap_2d(self):
        # Drone A and B are 15m apart, safety buffer is 10m. No overlap.
        potential_conflicts = check_spatial_proximity(
            self.mission_A.drone_id, self.traj_A,
            self.mission_B.drone_id, self.traj_B,
            self.safety_buffer
        )
        self.assertEqual(len(potential_conflicts), 0)

    def test_check_spatial_proximity_with_overlap_2d(self):
        # Drone A and C are 5m apart, safety buffer is 10m. Should have overlaps.
        potential_conflicts = check_spatial_proximity(
            self.mission_A.drone_id, self.traj_A,
            self.mission_C.drone_id, self.traj_C,
            self.safety_buffer
        )
        self.assertGreater(len(potential_conflicts), 0)
        # Verify at least one point is close to the expected 5m distance
        found_close_point = False
        for p_wp, s_wp in potential_conflicts:
            if calculate_distance(p_wp, s_wp) < self.safety_buffer:
                found_close_point = True
                break
        self.assertTrue(found_close_point)

    def test_check_spatial_proximity_head_on_overlap_2d(self):
        # Drone A and D are head-on. Many points should be close.
        potential_conflicts = check_spatial_proximity(
            self.mission_A.drone_id, self.traj_A,
            self.mission_D.drone_id, self.traj_D,
            self.safety_buffer
        )
        self.assertGreater(len(potential_conflicts), 0)
        # Check if the collision point (50,0) is represented
        collision_x_count = 0
        for p_wp, s_wp in potential_conflicts:
            if p_wp.x > 40 and p_wp.x < 60 and s_wp.x > 40 and s_wp.x < 60:
                collision_x_count += 1
        self.assertGreater(collision_x_count, 0)

    def test_check_spatial_proximity_3d_no_overlap(self):
        # Drone E and F are 5m apart in Z, safety buffer is 10m. Should have overlaps.
        # This will test if Z-dimension is considered correctly
        potential_conflicts = check_spatial_proximity(
            self.mission_E.drone_id, self.traj_E,
            self.mission_F.drone_id, self.traj_F,
            self.safety_buffer
        )
        self.assertGreater(len(potential_conflicts), 0)
        # Confirm the distance is within the safety buffer and Z-difference is 5m
        for p_wp, s_wp in potential_conflicts:
            self.assertLess(calculate_distance(p_wp, s_wp), self.safety_buffer)
            self.assertAlmostEqual(abs(p_wp.z - s_wp.z), 5.0)

    def test_check_spatial_proximity_invalid_trajectory_input(self):
        with self.assertRaises(TypeError):
            check_spatial_proximity("A", "not_a_list", "B", self.traj_B, self.safety_buffer)
        with self.assertRaises(TypeError):
            check_spatial_proximity("A", [Waypoint(0,0)], "B", [1,2,3], self.safety_buffer)

    def test_check_spatial_proximity_invalid_safety_buffer(self):
        with self.assertRaises(ValueError):
            check_spatial_proximity("A", self.traj_A, "B", self.traj_B, 0)
        with self.assertRaises(ValueError):
            check_spatial_proximity("A", self.traj_A, "B", self.traj_B, -5)

if __name__ == '__main__':
    unittest.main()