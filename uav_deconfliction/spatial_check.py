from typing import List, Tuple
import math

from uav_deconfliction.data_models import Waypoint, Mission, Conflict
from uav_deconfliction.trajectory_generation import interpolate_trajectory # We'll use this for dense paths

def calculate_distance(wp1: Waypoint, wp2: Waypoint) -> float:
    """Calculates the Euclidean distance between two Waypoint objects."""
    dx = wp1.x - wp2.x
    dy = wp1.y - wp2.y
    dz = wp1.z - wp2.z
    return math.sqrt(dx**2 + dy**2 + dz**2)

def check_spatial_proximity(
    primary_mission_id: str,
    primary_trajectory: List[Waypoint],
    simulated_mission_id: str,
    simulated_trajectory: List[Waypoint],
    safety_buffer: float
) -> List[Tuple[Waypoint, Waypoint]]:
    """
    Checks for spatial proximity between two pre-interpolated trajectories.
    Returns a list of tuples, where each tuple contains (primary_drone_waypoint, simulated_drone_waypoint)
    at the time of spatial conflict. These are potential conflict points.
    """
    if not isinstance(primary_trajectory, list) or not all(isinstance(wp, Waypoint) for wp in primary_trajectory):
        raise TypeError("primary_trajectory must be a list of Waypoint objects.")
    if not isinstance(simulated_trajectory, list) or not all(isinstance(wp, Waypoint) for wp in simulated_trajectory):
        raise TypeError("simulated_trajectory must be a list of Waypoint objects.")
    if not isinstance(safety_buffer, (int, float)) or safety_buffer <= 0:
        raise ValueError("safety_buffer must be a positive numeric value.")

    potential_spatial_conflicts: List[Tuple[Waypoint, Waypoint]] = []

    # Optimize by iterating through the shorter trajectory for the outer loop
    # This might not always be best if one trajectory is very short and the other very long
    # but for simplicity for now, we iterate over primary and check against simulated.
    # For very large scale, spatial indexing (e.g., k-d trees) would be used here.

    for p_wp in primary_trajectory:
        for s_wp in simulated_trajectory:
            # Only compare if they are roughly in the same time window to avoid unnecessary spatial checks
            # A more rigorous temporal check will follow in temporal_check.py
            # For now, a rough time check helps prune pairs that are far apart in time
            time_diff = abs((p_wp.time - s_wp.time).total_seconds())
            if time_diff < safety_buffer * 2: # Heuristic: if their times are too far apart, skip
                                              # (e.g., if safety buffer is 10m, and a drone moves 5m/s,
                                              # a time diff of 20s means they are ~100m apart temporally already)
                                              # This is a simplification; temporal_check will be the definitive one.
                distance = calculate_distance(p_wp, s_wp)
                if distance < safety_buffer:
                    # Found a potential spatial conflict at this time step
                    potential_spatial_conflicts.append((p_wp, s_wp))
    
    # Filter for unique (or representative) conflict points if many points are very close to each other
    # For simplicity, we'll keep all for now, as temporal_check will handle actual conflict validation.
    return potential_spatial_conflicts