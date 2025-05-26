from typing import List, Optional
import datetime
from uav_deconfliction.data_models import Waypoint, Mission

def interpolate_trajectory(mission: Mission, time_step_seconds: float = 1.0) -> List[Waypoint]:
    """
    Generates a densely sampled trajectory for a drone's mission by interpolating
    between waypoints at fixed time intervals.

    Args:
        mission (Mission): The drone's mission object.
        time_step_seconds (float): The time interval (in seconds) for sampling
                                   points along the trajectory.

    Returns:
        List[Waypoint]: A list of Waypoint objects representing the drone's
                        position at each sampled time step.
    """
    if not isinstance(mission, Mission):
        raise TypeError("Input must be a Mission object.")
    if time_step_seconds <= 0:
        raise ValueError("time_step_seconds must be a positive value.")

    interpolated_path: List[Waypoint] = []
    
    # Ensure mission has at least two waypoints for interpolation
    if len(mission.waypoints) < 2:
        if mission.waypoints:
            # If only one waypoint, the drone is static at that point for the mission duration
            current_time = mission.start_time
            while current_time <= mission.end_time:
                interpolated_path.append(
                    Waypoint(x=mission.waypoints[0].x, y=mission.waypoints[0].y,
                             z=mission.waypoints[0].z, time=current_time)
                )
                current_time += datetime.timedelta(seconds=time_step_seconds)
            # Ensure the very last point is at end_time if not already included
            if not interpolated_path or interpolated_path[-1].time < mission.end_time:
                interpolated_path.append(
                    Waypoint(x=mission.waypoints[0].x, y=mission.waypoints[0].y,
                             z=mission.waypoints[0].z, time=mission.end_time)
                )
        return interpolated_path

    current_waypoint_index = 0
    current_time = mission.start_time

    while current_time <= mission.end_time:
        # Find the two waypoints that bound the current_time
        # (current_waypoint_index and current_waypoint_index + 1)
        while current_waypoint_index + 1 < len(mission.waypoints) and \
              mission.waypoints[current_waypoint_index + 1].time <= current_time:
            current_waypoint_index += 1

        wp1 = mission.waypoints[current_waypoint_index]
        # If we are at or past the last waypoint's time, just take the last waypoint's position
        if current_waypoint_index == len(mission.waypoints) - 1:
            interpolated_path.append(Waypoint(x=wp1.x, y=wp1.y, z=wp1.z, time=current_time))
        else:
            wp2 = mission.waypoints[current_waypoint_index + 1]

            # Calculate the interpolation factor (0.0 at wp1.time, 1.0 at wp2.time)
            segment_duration = (wp2.time - wp1.time).total_seconds()
            
            # Handle cases where segment_duration is zero (e.g., waypoints with same time)
            # In this case, the drone is considered at wp2's position at wp1's time and after
            if segment_duration <= 0:
                interpolated_path.append(Waypoint(x=wp2.x, y=wp2.y, z=wp2.z, time=current_time))
            else:
                time_into_segment = (current_time - wp1.time).total_seconds()
                alpha = time_into_segment / segment_duration

                # Perform linear interpolation for x, y, and z coordinates
                interp_x = wp1.x + alpha * (wp2.x - wp1.x)
                interp_y = wp1.y + alpha * (wp2.y - wp1.y)
                interp_z = wp1.z + alpha * (wp2.z - wp1.z)

                interpolated_path.append(Waypoint(x=interp_x, y=interp_y, z=interp_z, time=current_time))
        
        current_time += datetime.timedelta(seconds=time_step_seconds)
    
    # Ensure the very last point of the mission (at mission.end_time) is included
    # This covers scenarios where the loop might stop just before end_time due to time_step_seconds
    if not interpolated_path or interpolated_path[-1].time < mission.end_time:
        final_waypoint_time = mission.end_time
        # Find the segment covering the end time
        final_wp1_idx = 0
        while final_wp1_idx + 1 < len(mission.waypoints) and \
              mission.waypoints[final_wp1_idx + 1].time <= final_waypoint_time:
            final_wp1_idx += 1
        
        wp_final_1 = mission.waypoints[final_wp1_idx]
        if final_wp1_idx == len(mission.waypoints) - 1:
            interpolated_path.append(Waypoint(x=wp_final_1.x, y=wp_final_1.y, z=wp_final_1.z, time=final_waypoint_time))
        else:
            wp_final_2 = mission.waypoints[final_wp1_idx + 1]
            segment_duration = (wp_final_2.time - wp_final_1.time).total_seconds()
            
            if segment_duration <= 0: # Should ideally not happen if mission times are reasonable
                interpolated_path.append(Waypoint(x=wp_final_2.x, y=wp_final_2.y, z=wp_final_2.z, time=final_waypoint_time))
            else:
                time_into_segment = (final_waypoint_time - wp_final_1.time).total_seconds()
                alpha = time_into_segment / segment_duration
                interp_x = wp_final_1.x + alpha * (wp_final_2.x - wp_final_1.x)
                interp_y = wp_final_1.y + alpha * (wp_final_2.y - wp_final_1.y)
                interp_z = wp_final_1.z + alpha * (wp_final_2.z - wp_final_1.z)
                interpolated_path.append(Waypoint(x=interp_x, y=interp_y, z=interp_z, time=final_waypoint_time))

    return interpolated_path

def get_position_at_time(mission: Mission, query_time: datetime.datetime) -> Optional[Waypoint]:
    """
    Retrieves the drone's interpolated position at a specific query time.

    Args:
        mission (Mission): The drone's mission object.
        query_time (datetime.datetime): The specific time to query the position for.

    Returns:
        Optional[Waypoint]: The interpolated Waypoint object at query_time, or None
                            if query_time is outside the mission's overall time window.
    """
    if not isinstance(mission, Mission):
        raise TypeError("Input 'mission' must be a Mission object.")
    if not isinstance(query_time, datetime.datetime):
        raise TypeError("Input 'query_time' must be a datetime object.")

    # Check if query_time is within the mission's overall time window
    if not (mission.start_time <= query_time <= mission.end_time):
        return None

    # Find the segment (wp1, wp2) that contains the query_time
    wp1: Waypoint = mission.waypoints[0]
    wp2: Optional[Waypoint] = None

    for i in range(len(mission.waypoints) - 1):
        if mission.waypoints[i].time <= query_time <= mission.waypoints[i+1].time:
            wp1 = mission.waypoints[i]
            wp2 = mission.waypoints[i+1]
            break
    
    # If query_time is exactly the last waypoint's time, or beyond the last segment but within mission end
    if query_time == mission.waypoints[-1].time:
        return mission.waypoints[-1]
    
    # If query_time is exactly the first waypoint's time
    if query_time == mission.waypoints[0].time:
        return mission.waypoints[0]

    # If wp2 is still None, it means query_time is beyond the last explicit waypoint's time
    # but still within the mission's end_time. In this case, we assume the drone stays at the
    # last waypoint's position until mission.end_time.
    if wp2 is None:
        # This occurs if query_time is between the last waypoint's time and mission.end_time
        # (e.g., if waypoints didn't go all the way to mission.end_time, but mission.end_time was set later)
        # Or, if the loop logic missed it for a very specific time.
        # Given our _assign_waypoint_times_if_needed logic in Mission, wp.time should span start_time to end_time
        # However, to be robust, if we reach here, it implies query_time might be near the end or exactly end_time.
        # We can assume it's at the last recorded position.
        return mission.waypoints[-1] # Assume drone is at its last known waypoint

    # Calculate interpolation factor
    segment_duration = (wp2.time - wp1.time).total_seconds()
    if segment_duration <= 0: # Handles case where two consecutive waypoints have the same time
        return wp1 # Or wp2, they are at the same location

    time_into_segment = (query_time - wp1.time).total_seconds()
    alpha = time_into_segment / segment_duration

    # Interpolate coordinates
    interp_x = wp1.x + alpha * (wp2.x - wp1.x)
    interp_y = wp1.y + alpha * (wp2.y - wp1.y)
    interp_z = wp1.z + alpha * (wp2.z - wp1.z)

    return Waypoint(x=interp_x, y=interp_y, z=interp_z, time=query_time)