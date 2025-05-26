import datetime
import math

class Waypoint:
    """
    Represents a single point in space and time for a drone's trajectory.
    Can be 2D (x, y) or 3D (x, y, z).
    """
    def __init__(self, x: float, y: float, z: float = 0.0, time: datetime.datetime = None):
        if not isinstance(x, (int, float)) or \
           not isinstance(y, (int, float)) or \
           not isinstance(z, (int, float)):
            raise ValueError("Coordinates (x, y, z) must be numeric.")
        if time is not None and not isinstance(time, datetime.datetime):
            raise ValueError("Time must be a datetime object or None.")

        self.x = x
        self.y = y
        self.z = z
        self.time = time

    def to_tuple(self, include_time: bool = False) -> tuple:
        """Returns the waypoint as a tuple (x, y) or (x, y, z). Optionally includes time."""
        if include_time:
            return (self.x, self.y, self.z, self.time)
        return (self.x, self.y, self.z) if self.z != 0.0 else (self.x, self.y)

    def distance_to(self, other_waypoint: 'Waypoint') -> float:
        """Calculates the Euclidean distance to another waypoint."""
        if not isinstance(other_waypoint, Waypoint):
            raise TypeError("Can only calculate distance to another Waypoint object.")
        
        dx = self.x - other_waypoint.x
        dy = self.y - other_waypoint.y
        dz = self.z - other_waypoint.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def __repr__(self):
        time_str = f", time={self.time.strftime('%H:%M:%S')}" if self.time else ""
        if self.z != 0.0:
            return f"Waypoint(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}{time_str})"
        return f"Waypoint(x={self.x:.2f}, y={self.y:.2f}{time_str})"

    def __eq__(self, other):
        if not isinstance(other, Waypoint):
            return NotImplemented
        return (self.x == other.x and self.y == other.y and self.z == other.z and self.time == other.time)

    def __hash__(self):
        return hash((self.x, self.y, self.z, self.time))

class Mission:
    """
    Represents a drone's planned mission, defined by a series of waypoints
    and an overall start and end time.
    """
    def __init__(self, drone_id: str, waypoints: list[Waypoint],
                 start_time: datetime.datetime, end_time: datetime.datetime):
        if not isinstance(drone_id, str) or not drone_id:
            raise ValueError("Drone ID must be a non-empty string.")
        if not isinstance(waypoints, list) or not all(isinstance(wp, Waypoint) for wp in waypoints) or len(waypoints) < 2:
            raise ValueError("Waypoints must be a list of at least two Waypoint objects.")
        if not isinstance(start_time, datetime.datetime) or not isinstance(end_time, datetime.datetime):
            raise ValueError("Start time and end time must be datetime objects.")
        if start_time >= end_time:
            raise ValueError("Start time must be before end time.")

        self.drone_id = drone_id
        self.waypoints = waypoints
        self.start_time = start_time
        self.end_time = end_time

        # Ensure waypoints have times, if not, attempt to distribute times linearly
        self._assign_waypoint_times_if_needed()

    def _assign_waypoint_times_if_needed(self):
        """
        If waypoints in the mission don't have individual times,
        assign them linearly based on mission start/end times and path length.
        """
        if any(wp.time is None for wp in self.waypoints):
            total_duration_seconds = (self.end_time - self.start_time).total_seconds()
            
            # Calculate total path distance to proportionally distribute time
            total_path_distance = 0.0
            for i in range(len(self.waypoints) - 1):
                total_path_distance += self.waypoints[i].distance_to(self.waypoints[i+1])
            
            if total_path_distance == 0: # All waypoints are the same, distribute time evenly
                for i, wp in enumerate(self.waypoints):
                    fraction = i / (len(self.waypoints) - 1) if len(self.waypoints) > 1 else 0
                    wp.time = self.start_time + datetime.timedelta(seconds=total_duration_seconds * fraction)
            else:
                current_distance = 0.0
                self.waypoints[0].time = self.start_time # First waypoint is at mission start
                for i in range(1, len(self.waypoints)):
                    segment_distance = self.waypoints[i-1].distance_to(self.waypoints[i])
                    current_distance += segment_distance
                    # Proportional time assignment based on distance covered
                    fraction_of_total_distance = current_distance / total_path_distance
                    self.waypoints[i].time = self.start_time + datetime.timedelta(seconds=total_duration_seconds * fraction_of_total_distance)
            
            # Ensure the last waypoint's time is exactly the end_time
            if self.waypoints:
                self.waypoints[-1].time = self.end_time


    def get_duration(self) -> datetime.timedelta:
        """Returns the total duration of the mission."""
        return self.end_time - self.start_time

    def get_path_length(self) -> float:
        """Calculates the total length of the mission path."""
        length = 0.0
        for i in range(len(self.waypoints) - 1):
            length += self.waypoints[i].distance_to(self.waypoints[i+1])
        return length

    def __repr__(self):
        return (f"Mission(drone_id='{self.drone_id}', "
                f"waypoints={len(self.waypoints)} points, "
                f"start_time={self.start_time.strftime('%Y-%m-%d %H:%M:%S')}, "
                f"end_time={self.end_time.strftime('%Y-%m-%d %H:%M:%S')})")

class Conflict:
    """
    Represents a detected conflict between drones in space and time.
    """
    def __init__(self, location: Waypoint, time: datetime.datetime,
                 conflicting_drone_ids: list[str], description: str):
        if not isinstance(location, Waypoint):
            raise ValueError("Conflict location must be a Waypoint object.")
        if not isinstance(time, datetime.datetime):
            raise ValueError("Conflict time must be a datetime object.")
        if not isinstance(conflicting_drone_ids, list) or not all(isinstance(id, str) for id in conflicting_drone_ids):
            raise ValueError("Conflicting drone IDs must be a list of strings.")
        if not isinstance(description, str) or not description:
            raise ValueError("Description must be a non-empty string.")

        self.location = location
        self.time = time
        self.conflicting_drone_ids = sorted(list(set(conflicting_drone_ids))) # Ensure unique and sorted
        self.description = description

    def to_dict(self) -> dict:
        """Returns the conflict details as a dictionary."""
        return {
            "location": self.location.to_tuple(include_time=True),
            "time": self.time.isoformat(),
            "conflicting_drone_ids": self.conflicting_drone_ids,
            "description": self.description
        }

    def __repr__(self):
        loc_str = f"({self.location.x:.2f}, {self.location.y:.2f}"
        if self.location.z != 0.0:
            loc_str += f", {self.location.z:.2f}"
        loc_str += ")"
        time_str = self.time.strftime('%Y-%m-%d %H:%M:%S')
        
        return (f"Conflict(location={loc_str}, time={time_str}, "
                f"drones={self.conflicting_drone_ids}, desc='{self.description}')")