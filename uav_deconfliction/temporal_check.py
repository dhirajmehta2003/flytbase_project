from typing import List, Tuple, Optional
import datetime
from uav_deconfliction.data_models import Waypoint, Mission, Conflict
from uav_deconfliction.trajectory_generation import get_position_at_time
from uav_deconfliction.spatial_check import calculate_distance # Reusing distance calculation

def check_temporal_overlap_at_location(
    primary_mission: Mission,
    simulated_mission: Mission,
    potential_conflict_point_primary: Waypoint, # A waypoint from primary_trajectory that's spatially close
    potential_conflict_point_simulated: Waypoint, # A waypoint from simulated_trajectory that's spatially close
    safety_buffer: float,
    time_proximity_threshold: datetime.timedelta = datetime.timedelta(seconds=5) # How close in time to consider a conflict
) -> Optional[Conflict]:
    """
    Refines a potential spatial conflict by verifying if the drones are in
    the same spatial area at (approximately) the same time.

    Args:
        primary_mission (Mission): The primary drone's mission object.
        simulated_mission (Mission): The simulated drone's mission object.
        potential_conflict_point_primary (Waypoint): A Waypoint from the primary's
                                                      interpolated trajectory where proximity was found.
        potential_conflict_point_simulated (Waypoint): A Waypoint from the simulated drone's
                                                        interpolated trajectory where proximity was found.
        safety_buffer (float): The minimum distance threshold for spatial conflict.
        time_proximity_threshold (datetime.timedelta): Defines a small time window around
                                                       the potential conflict time to check.

    Returns:
        Optional[Conflict]: A Conflict object if a true spatio-temporal conflict is detected,
                            otherwise None.
    """
    if not isinstance(primary_mission, Mission) or not isinstance(simulated_mission, Mission):
        raise TypeError("Mission inputs must be Mission objects.")
    if not isinstance(potential_conflict_point_primary, Waypoint) or \
       not isinstance(potential_conflict_point_simulated, Waypoint):
        raise TypeError("Potential conflict points must be Waypoint objects.")
    if not isinstance(safety_buffer, (int, float)) or safety_buffer <= 0:
        raise ValueError("safety_buffer must be a positive numeric value.")
    if not isinstance(time_proximity_threshold, datetime.timedelta) or \
       time_proximity_threshold.total_seconds() < 0:
        raise ValueError("time_proximity_threshold must be a non-negative timedelta.")

    # The 'time' attribute of the potential_conflict_point_primary is when the primary drone
    # is at that spatial location. We need to check if the simulated drone is also there
    # around that same time.

    # Define a time window around the primary drone's conflict time
    check_time_start = potential_conflict_point_primary.time - time_proximity_threshold
    check_time_end = potential_conflict_point_primary.time + time_proximity_threshold

    # Iterate through small time steps within this window
    # We choose a small step here, perhaps smaller than the `time_step_seconds` used for full trajectory generation,
    # to be more precise around the conflict point.
    # For a more robust check, we could do more complex interval overlap analysis.
    current_check_time = max(check_time_start, primary_mission.start_time, simulated_mission.start_time)
    
    # Iterate as long as we are within the checking window and within both missions' times
    while current_check_time <= min(check_time_end, primary_mission.end_time, simulated_mission.end_time):
        
        # Get interpolated positions for both drones at this exact time
        pos_primary = get_position_at_time(primary_mission, current_check_time)
        pos_simulated = get_position_at_time(simulated_mission, current_check_time)

        # Ensure both drones are active at this time (not None)
        if pos_primary and pos_simulated:
            distance_at_time = calculate_distance(pos_primary, pos_simulated)
            
            if distance_at_time < safety_buffer:
                # A true spatio-temporal conflict detected!
                conflict_location = Waypoint(
                    x=(pos_primary.x + pos_simulated.x) / 2,
                    y=(pos_primary.y + pos_simulated.y) / 2,
                    z=(pos_primary.z + pos_simulated.z) / 2,
                    time=current_check_time # The time of conflict
                )
                conflicting_ids = [primary_mission.drone_id, simulated_mission.drone_id]
                description = (f"Spatio-temporal conflict between {primary_mission.drone_id} and "
                               f"{simulated_mission.drone_id} at {conflict_location.to_tuple(include_time=False)} "
                               f"at {current_check_time.strftime('%H:%M:%S')}.")
                return Conflict(conflict_location, current_check_time, conflicting_ids, description)
        
        current_check_time += datetime.timedelta(seconds=1) # Check every second within the window

    return None # No spatio-temporal conflict found within the specified time window