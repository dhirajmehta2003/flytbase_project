import unittest
import datetime
import math
from uav_deconfliction.data_models import Waypoint, Mission, Conflict

class TestDataModels(unittest.TestCase):

    def setUp(self):
        self.now = datetime.datetime.now().replace(microsecond=0) # Round to second for easier comparison

    # --- Test Waypoint Class ---
    def test_waypoint_initialization_2d(self):
        wp = Waypoint(x=10, y=20)
        self.assertEqual(wp.x, 10)
        self.assertEqual(wp.y, 20)
        self.assertEqual(wp.z, 0.0)
        self.assertIsNone(wp.time)

    def test_waypoint_initialization_3d(self):
        t = self.now + datetime.timedelta(seconds=10)
        wp = Waypoint(x=10, y=20, z=30, time=t)
        self.assertEqual(wp.x, 10)
        self.assertEqual(wp.y, 20)
        self.assertEqual(wp.z, 30)
        self.assertEqual(wp.time, t)

    def test_waypoint_invalid_coordinates(self):
        with self.assertRaises(ValueError):
            Waypoint(x="abc", y=20)
        with self.assertRaises(ValueError):
            Waypoint(x=10, y=None)

    def test_waypoint_invalid_time(self):
        with self.assertRaises(ValueError):
            Waypoint(x=10, y=20, time="not_a_datetime")

    def test_waypoint_to_tuple(self):
        wp_2d = Waypoint(1, 2)
        self.assertEqual(wp_2d.to_tuple(), (1, 2))
        self.assertEqual(wp_2d.to_tuple(include_time=True), (1, 2, 0.0, None)) # 2D default z is 0.0

        t = self.now
        wp_3d = Waypoint(1, 2, 3, t)
        self.assertEqual(wp_3d.to_tuple(), (1, 2, 3))
        self.assertEqual(wp_3d.to_tuple(include_time=True), (1, 2, 3, t))

    def test_waypoint_distance_to(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(3, 4, 0)
        self.assertAlmostEqual(wp1.distance_to(wp2), 5.0)

        wp3 = Waypoint(0, 0, 5)
        self.assertAlmostEqual(wp1.distance_to(wp3), 5.0)

        wp4 = Waypoint(3, 4, 5)
        self.assertAlmostEqual(wp1.distance_to(wp4), math.sqrt(3**2 + 4**2 + 5**2)) # sqrt(50)

        with self.assertRaises(TypeError):
            wp1.distance_to("not_a_waypoint")

    def test_waypoint_equality(self):
        t1 = self.now
        t2 = self.now + datetime.timedelta(seconds=1)
        wp1 = Waypoint(1, 2, 3, t1)
        wp2 = Waypoint(1, 2, 3, t1)
        wp3 = Waypoint(1, 2, 4, t1)
        wp4 = Waypoint(1, 2, 3, t2)
        
        self.assertTrue(wp1 == wp2)
        self.assertFalse(wp1 == wp3)
        self.assertFalse(wp1 == wp4)
        self.assertFalse(wp1 == "not_a_waypoint")


    # --- Test Mission Class ---
    def test_mission_initialization_valid(self):
        wp1 = Waypoint(0, 0, time=self.now)
        wp2 = Waypoint(100, 100, time=self.now + datetime.timedelta(minutes=5))
        mission = Mission("DRONE_TEST", [wp1, wp2], self.now, self.now + datetime.timedelta(minutes=5))
        self.assertEqual(mission.drone_id, "DRONE_TEST")
        self.assertEqual(len(mission.waypoints), 2)
        self.assertEqual(mission.start_time, self.now)
        self.assertEqual(mission.end_time, self.now + datetime.timedelta(minutes=5))

    def test_mission_invalid_drone_id(self):
        wp1 = Waypoint(0, 0, time=self.now)
        wp2 = Waypoint(100, 100, time=self.now + datetime.timedelta(minutes=5))
        with self.assertRaises(ValueError):
            Mission("", [wp1, wp2], self.now, self.now + datetime.timedelta(minutes=5))

    def test_mission_invalid_waypoints(self):
        with self.assertRaises(ValueError): # Less than 2 waypoints
            Mission("DRONE_TEST", [Waypoint(0,0)], self.now, self.now + datetime.timedelta(minutes=5))
        with self.assertRaises(ValueError): # Not list of Waypoint objects
            Mission("DRONE_TEST", ["invalid"], self.now, self.now + datetime.timedelta(minutes=5))

    def test_mission_invalid_times(self):
        wp1 = Waypoint(0, 0, time=self.now)
        wp2 = Waypoint(100, 100, time=self.now + datetime.timedelta(minutes=5))
        with self.assertRaises(ValueError): # End time before start time
            Mission("DRONE_TEST", [wp1, wp2], self.now, self.now - datetime.timedelta(minutes=1))
        with self.assertRaises(ValueError): # Not datetime objects
            Mission("DRONE_TEST", [wp1, wp2], "not_datetime", self.now + datetime.timedelta(minutes=5))

    def test_mission_assign_waypoint_times_if_needed_no_times(self):
        wp1 = Waypoint(0, 0)
        wp2 = Waypoint(100, 0)
        mission = Mission("DRONE_TIMES", [wp1, wp2], self.now, self.now + datetime.timedelta(minutes=10))
        self.assertEqual(mission.waypoints[0].time, self.now)
        self.assertEqual(mission.waypoints[1].time, self.now + datetime.timedelta(minutes=10))

    def test_mission_assign_waypoint_times_if_needed_partial_times(self):
        wp1 = Waypoint(0, 0, time=self.now) # This one has time
        wp2 = Waypoint(100, 0) # This one does not
        wp3 = Waypoint(200, 0) # This one does not
        mission = Mission("DRONE_PARTIAL", [wp1, wp2, wp3], self.now, self.now + datetime.timedelta(minutes=10))
        self.assertEqual(mission.waypoints[0].time, self.now)
        self.assertEqual(mission.waypoints[2].time, self.now + datetime.timedelta(minutes=10))
        
        # Midpoint should be at half time (approx, due to linear path)
        mid_time = self.now + datetime.timedelta(minutes=5)
        self.assertAlmostEqual(mission.waypoints[1].time.timestamp(), mid_time.timestamp(), delta=1)
        
    def test_mission_get_duration(self):
        wp1 = Waypoint(0, 0, time=self.now)
        wp2 = Waypoint(100, 100, time=self.now + datetime.timedelta(minutes=5))
        mission = Mission("DRONE_TEST", [wp1, wp2], self.now, self.now + datetime.timedelta(minutes=5))
        self.assertEqual(mission.get_duration(), datetime.timedelta(minutes=5))

    def test_mission_get_path_length(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(3, 4, 0)
        wp3 = Waypoint(3, 4, 5)
        mission = Mission("DRONE_PATH", [wp1, wp2, wp3], self.now, self.now + datetime.timedelta(minutes=10))
        # Distance wp1-wp2 = 5.0
        # Distance wp2-wp3 = 5.0
        self.assertAlmostEqual(mission.get_path_length(), 10.0)


    # --- Test Conflict Class ---
    def test_conflict_initialization_valid(self):
        t = self.now + datetime.timedelta(minutes=1)
        loc = Waypoint(50, 50, 10, t)
        conflict = Conflict(loc, t, ["DRONE_A", "DRONE_B"], "Collision detected")
        self.assertEqual(conflict.location, loc)
        self.assertEqual(conflict.time, t)
        self.assertEqual(conflict.conflicting_drone_ids, ["DRONE_A", "DRONE_B"]) # Should be sorted
        self.assertEqual(conflict.description, "Collision detected")

    def test_conflict_invalid_location(self):
        with self.assertRaises(ValueError):
            Conflict("not_a_waypoint", self.now, ["A", "B"], "Desc")

    def test_conflict_invalid_time(self):
        loc = Waypoint(0,0)
        with self.assertRaises(ValueError):
            Conflict(loc, "not_datetime", ["A", "B"], "Desc")

    def test_conflict_invalid_drone_ids(self):
        loc = Waypoint(0,0)
        with self.assertRaises(ValueError): # Not list
            Conflict(loc, self.now, "not_list", "Desc")
        with self.assertRaises(ValueError): # Not strings in list
            Conflict(loc, self.now, ["A", 123], "Desc")

    def test_conflict_invalid_description(self):
        loc = Waypoint(0,0)
        with self.assertRaises(ValueError):
            Conflict(loc, self.now, ["A", "B"], "")

    def test_conflict_to_dict(self):
        t = self.now + datetime.timedelta(minutes=1)
        loc = Waypoint(50, 50, 10, t)
        conflict = Conflict(loc, t, ["DRONE_A", "DRONE_B"], "Collision detected")
        
        expected_dict = {
            "location": (50, 50, 10, t),
            "time": t.isoformat(),
            "conflicting_drone_ids": ["DRONE_A", "DRONE_B"],
            "description": "Collision detected"
        }
        self.assertEqual(conflict.to_dict(), expected_dict)

    def test_conflict_drone_id_sorting_and_uniqueness(self):
        t = self.now
        loc = Waypoint(0,0,0,t)
        conflict = Conflict(loc, t, ["DRONE_C", "DRONE_A", "DRONE_C"], "Test")
        self.assertEqual(conflict.conflicting_drone_ids, ["DRONE_A", "DRONE_C"])


if __name__ == '__main__':
    unittest.main()