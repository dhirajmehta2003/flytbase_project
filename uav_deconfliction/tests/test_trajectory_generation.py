import unittest
import datetime
from uav_deconfliction.data_models import Waypoint, Mission
from uav_deconfliction.trajectory_generation import interpolate_trajectory, get_position_at_time

class TestTrajectoryGeneration(unittest.TestCase):

    def setUp(self):
        self.now = datetime.datetime.now().replace(microsecond=0)

        # 2D Linear Mission (100 units in 5 minutes)
        self.wp_start_2d = Waypoint(0, 0, time=self.now)
        self.wp_end_2d = Waypoint(100, 0, time=self.now + datetime.timedelta(minutes=5))
        self.mission_2d = Mission("DRONE_2D", [self.wp_start_2d, self.wp_end_2d], 
                                  self.now, self.now + datetime.timedelta(minutes=5))

        # 3D Mission (Ascending, 100 units in 5 minutes)
        self.wp_start_3d = Waypoint(0, 0, 0, time=self.now)
        self.wp_mid_3d = Waypoint(50, 50, 50, time=self.now + datetime.timedelta(minutes=2, seconds=30))
        self.wp_end_3d = Waypoint(100, 100, 100, time=self.now + datetime.timedelta(minutes=5))
        self.mission_3d = Mission("DRONE_3D", [self.wp_start_3d, self.wp_mid_3d, self.wp_end_3d],
                                  self.now, self.now + datetime.timedelta(minutes=5))
        
        # Mission with Waypoints having no time, relying on Mission auto-assignment
        self.wp_no_time_1 = Waypoint(0,0)
        self.wp_no_time_2 = Waypoint(200,0)
        self.mission_no_times = Mission("DRONE_NO_TIMES", [self.wp_no_time_1, self.wp_no_time_2],
                                        self.now, self.now + datetime.timedelta(minutes=10))


    # --- Test interpolate_trajectory ---
    def test_interpolate_trajectory_2d_basic(self):
        # Sample every 1 minute for a 5-minute mission
        trajectory = interpolate_trajectory(self.mission_2d, time_step_seconds=60)
        self.assertEqual(len(trajectory), 6) # At 0, 1, 2, 3, 4, 5 minutes

        self.assertAlmostEqual(trajectory[0].x, 0)
        self.assertAlmostEqual(trajectory[0].y, 0)
        self.assertEqual(trajectory[0].time, self.now)

        self.assertAlmostEqual(trajectory[1].x, 20) # At 1 min, 20% of 100
        self.assertAlmostEqual(trajectory[1].y, 0)
        self.assertEqual(trajectory[1].time, self.now + datetime.timedelta(minutes=1))

        self.assertAlmostEqual(trajectory[5].x, 100) # At 5 min, 100% of 100
        self.assertAlmostEqual(trajectory[5].y, 0)
        self.assertEqual(trajectory[5].time, self.now + datetime.timedelta(minutes=5))

    def test_interpolate_trajectory_3d_basic(self):
        # Sample every 1 minute for a 5-minute mission
        trajectory = interpolate_trajectory(self.mission_3d, time_step_seconds=60)
        self.assertEqual(len(trajectory), 6) # At 0, 1, 2, 3, 4, 5 minutes

        self.assertAlmostEqual(trajectory[0].x, 0)
        self.assertAlmostEqual(trajectory[0].y, 0)
        self.assertAlmostEqual(trajectory[0].z, 0)
        self.assertEqual(trajectory[0].time, self.now)

        # Interpolate between wp_start_3d and wp_mid_3d
        # At 1 min, 40% into first segment (1 min out of 2.5 min = 0.4)
        self.assertAlmostEqual(trajectory[1].x, 0 + 0.4 * (50-0)) # 20
        self.assertAlmostEqual(trajectory[1].y, 0 + 0.4 * (50-0)) # 20
        self.assertAlmostEqual(trajectory[1].z, 0 + 0.4 * (50-0)) # 20
        self.assertEqual(trajectory[1].time, self.now + datetime.timedelta(minutes=1))

        # At 5 min, should be at wp_end_3d
        self.assertAlmostEqual(trajectory[5].x, 100)
        self.assertAlmostEqual(trajectory[5].y, 100)
        self.assertAlmostEqual(trajectory[5].z, 100)
        self.assertEqual(trajectory[5].time, self.now + datetime.timedelta(minutes=5))

    def test_interpolate_trajectory_single_waypoint_mission(self):
        # Drone stays static at a single point for the mission duration
        # Missions require at least two waypoints. For static, use start and end at same point.
        single_wp_start = Waypoint(10, 20, 30, time=self.now)
        single_wp_end = Waypoint(10, 20, 30, time=self.now + datetime.timedelta(minutes=1))
        mission_static = Mission("StaticDrone", [single_wp_start, single_wp_end], self.now, self.now + datetime.timedelta(minutes=1))
        
        traj = interpolate_trajectory(mission_static, time_step_seconds=10)
        self.assertEqual(len(traj), 7) # 0s, 10s, 20s, ..., 60s
        for wp in traj:
            self.assertAlmostEqual(wp.x, 10)
            self.assertAlmostEqual(wp.y, 20)
            self.assertAlmostEqual(wp.z, 30)
        self.assertEqual(traj[-1].time, self.now + datetime.timedelta(minutes=1))

    def test_interpolate_trajectory_zero_duration_segment(self):
        # Waypoints at same time, drone should "jump" instantly
        wp1 = Waypoint(0,0,0, self.now)
        wp2 = Waypoint(100,0,0, self.now) # Same time as wp1
        wp3 = Waypoint(200,0,0, self.now + datetime.timedelta(minutes=1))
        mission_jump = Mission("JumpDrone", [wp1, wp2, wp3], self.now, self.now + datetime.timedelta(minutes=1))
        
        traj = interpolate_trajectory(mission_jump, time_step_seconds=10)
        # First point should be at wp1/wp2 (100,0,0)
        self.assertAlmostEqual(traj[0].x, 100)
        self.assertAlmostEqual(traj[0].y, 0)
        self.assertAlmostEqual(traj[0].z, 0)
        self.assertEqual(traj[0].time, self.now)
        
        # At 10 seconds, it should be at 100 + (10/60) * 100
        self.assertAlmostEqual(traj[1].x, 100 + (10/60) * 100)
        self.assertAlmostEqual(traj[1].y, 0)
        self.assertAlmostEqual(traj[1].z, 0)


    # --- Test get_position_at_time ---
    def test_get_position_at_time_inside_segment_2d(self):
        query_time = self.now + datetime.timedelta(minutes=2, seconds=30) # Halfway through mission
        pos = get_position_at_time(self.mission_2d, query_time)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos.x, 50.0)
        self.assertAlmostEqual(pos.y, 0.0)
        self.assertEqual(pos.time, query_time)

    def test_get_position_at_time_inside_segment_3d(self):
        query_time_segment1 = self.now + datetime.timedelta(minutes=1) # 1 min into 2.5 min first segment
        pos1 = get_position_at_time(self.mission_3d, query_time_segment1)
        self.assertIsNotNone(pos1)
        self.assertAlmostEqual(pos1.x, 20.0)
        self.assertAlmostEqual(pos1.y, 20.0)
        self.assertAlmostEqual(pos1.z, 20.0)
        self.assertEqual(pos1.time, query_time_segment1)

        query_time_segment2 = self.now + datetime.timedelta(minutes=4) # 1.5 min into 2.5 min second segment
        pos2 = get_position_at_time(self.mission_3d, query_time_segment2)
        self.assertIsNotNone(pos2)
        # alpha = (4m - 2.5m) / (5m - 2.5m) = 1.5 / 2.5 = 0.6
        self.assertAlmostEqual(pos2.x, self.wp_mid_3d.x + 0.6 * (self.wp_end_3d.x - self.wp_mid_3d.x)) # 50 + 0.6 * 50 = 80
        self.assertAlmostEqual(pos2.y, self.wp_mid_3d.y + 0.6 * (self.wp_end_3d.y - self.wp_mid_3d.y)) # 50 + 0.6 * 50 = 80
        self.assertAlmostEqual(pos2.z, self.wp_mid_3d.z + 0.6 * (self.wp_end_3d.z - self.wp_mid_3d.z)) # 50 + 0.6 * 50 = 80
        self.assertEqual(pos2.time, query_time_segment2)


    def test_get_position_at_time_at_waypoint_time(self):
        pos_start = get_position_at_time(self.mission_2d, self.now)
        self.assertIsNotNone(pos_start)
        self.assertAlmostEqual(pos_start.x, 0)
        self.assertAlmostEqual(pos_start.y, 0)

        pos_end = get_position_at_time(self.mission_2d, self.now + datetime.timedelta(minutes=5))
        self.assertIsNotNone(pos_end)
        self.assertAlmostEqual(pos_end.x, 100)
        self.assertAlmostEqual(pos_end.y, 0)

        pos_mid_3d = get_position_at_time(self.mission_3d, self.now + datetime.timedelta(minutes=2, seconds=30))
        self.assertIsNotNone(pos_mid_3d)
        self.assertAlmostEqual(pos_mid_3d.x, 50)
        self.assertAlmostEqual(pos_mid_3d.y, 50)
        self.assertAlmostEqual(pos_mid_3d.z, 50)


    def test_get_position_at_time_outside_mission_window(self):
        pos_before = get_position_at_time(self.mission_2d, self.now - datetime.timedelta(seconds=1))
        self.assertIsNone(pos_before)

        pos_after = get_position_at_time(self.mission_2d, self.now + datetime.timedelta(minutes=5, seconds=1))
        self.assertIsNone(pos_after)

    def test_get_position_at_time_invalid_inputs(self):
        with self.assertRaises(TypeError):
            get_position_at_time("not_a_mission", self.now)
        with self.assertRaises(TypeError):
            get_position_at_time(self.mission_2d, "not_a_datetime")

    def test_get_position_at_time_single_waypoint_mission(self):
        # Missions require at least two waypoints. For static, use start and end at same point.
        single_wp_start = Waypoint(10, 20, 30, time=self.now)
        single_wp_end = Waypoint(10, 20, 30, time=self.now + datetime.timedelta(minutes=1))
        mission_static = Mission("StaticDrone", [single_wp_start, single_wp_end], self.now, self.now + datetime.timedelta(minutes=1))
        query_time = self.now + datetime.timedelta(seconds=30)
        pos = get_position_at_time(mission_static, query_time)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos.x, 10)
        self.assertAlmostEqual(pos.y, 20)
        self.assertAlmostEqual(pos.z, 30)
        self.assertEqual(pos.time, query_time)
        
        # Test edge of static mission duration
        pos_end_time = get_position_at_time(mission_static, self.now + datetime.timedelta(minutes=1))
        self.assertIsNotNone(pos_end_time)
        self.assertAlmostEqual(pos_end_time.x, 10)


if __name__ == '__main__':
    unittest.main()