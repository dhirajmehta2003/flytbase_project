from typing import List, Dict, Union
import datetime
from uav_deconfliction.data_models import Waypoint, Mission, Conflict
from uav_deconfliction.trajectory_generation import interpolate_trajectory
from uav_deconfliction.spatial_check import check_spatial_proximity, calculate_distance
from uav_deconfliction.temporal_check import check_temporal_overlap_at_location

class DeconflictionSystem:
    """
    The central system for verifying a primary drone's mission against
    a set of simulated drone flight schedules.
    """
    def __init__(self, simulated_missions: List[Mission], safety_buffer: float = 10.0):
        if not isinstance(simulated_missions, list) or not all(isinstance(m, Mission) for m in simulated_missions):
            raise TypeError("simulated_missions must be a list of Mission objects.")
        if not isinstance(safety_buffer, (int, float)) or safety_buffer <= 0:
            raise ValueError("safety_buffer must be a positive numeric value.")

        self.simulated_missions = simulated_missions
        self.safety_buffer = safety_buffer
        # Pre-interpolate all simulated trajectories for efficiency
        self._pre_interpolated_simulated_trajectories = {
            m.drone_id: interpolate_trajectory(m) for m in simulated_missions
        }

    def verify_mission(self, primary_mission: Mission) -> Dict[str, Union[str, List[Conflict]]]:
        """
        Verifies if the primary drone's mission is safe by checking for spatio-temporal
        conflicts against all simulated flights.

        Args:
            primary_mission (Mission): The primary drone's mission to verify.

        Returns:
            Dict[str, Union[str, List[Conflict]]]: A dictionary containing the status
                                                   ("clear" or "conflict detected")
                                                   and a list of Conflict objects if conflicts exist.
        """
        if not isinstance(primary_mission, Mission):
            raise TypeError("primary_mission must be a Mission object.")

        detected_conflicts: List[Conflict] = []

        # Interpolate the primary drone's trajectory once
        primary_trajectory = interpolate_trajectory(primary_mission)

        for simulated_mission in self.simulated_missions:
            simulated_trajectory = self._pre_interpolated_simulated_trajectories[simulated_mission.drone_id]

            # 1. Spatial Check: Find potential spatial overlaps at any time
            # This returns (primary_wp, simulated_wp) pairs that are spatially too close
            potential_spatial_overlaps = check_spatial_proximity(
                primary_mission.drone_id, primary_trajectory,
                simulated_mission.drone_id, simulated_trajectory,
                self.safety_buffer
            )

            # 2. Temporal Check: Refine potential overlaps into true spatio-temporal conflicts
            # This iterates through the potential spatial overlaps
            for p_wp_potential, s_wp_potential in potential_spatial_overlaps:
                # Use the primary drone's waypoint time as the anchor for temporal check
                conflict = check_temporal_overlap_at_location(
                    primary_mission,
                    simulated_mission,
                    p_wp_potential,
                    s_wp_potential,
                    self.safety_buffer
                )
                if conflict:
                    detected_conflicts.append(conflict)

        if detected_conflicts:
            return {
                "status": "conflict detected",
                "conflict_details": detected_conflicts
            }
        else:
            return {
                "status": "clear",
                "conflict_details": []
            }