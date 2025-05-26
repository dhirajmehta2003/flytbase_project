import datetime
from typing import List
from uav_deconfliction.data_models import Waypoint, Mission
from uav_deconfliction.deconfliction_system import DeconflictionSystem
from uav_deconfliction.visualization import plot_trajectories_2d, plot_trajectories_3d, animate_trajectories_4d
import os

def create_scenario_1_conflict_free() -> tuple[Mission, list[Mission]]:
    """Scenario: Drones flying far apart, no conflict."""
    current_datetime = datetime.datetime.now()

    # Primary Drone (Drone A) - 2D mission
    waypoints_a = [
        Waypoint(x=0, y=0, time=current_datetime),
        Waypoint(x=100, y=0, time=current_datetime + datetime.timedelta(minutes=5))
    ]
    mission_a = Mission("DroneA", waypoints_a, current_datetime, current_datetime + datetime.timedelta(minutes=5))

    # Simulated Drone (Drone B) - 2D mission, parallel path, no overlap
    waypoints_b = [
        Waypoint(x=0, y=50, time=current_datetime),
        Waypoint(x=100, y=50, time=current_datetime + datetime.timedelta(minutes=5))
    ]
    mission_b = Mission("DroneB", waypoints_b, current_datetime, current_datetime + datetime.timedelta(minutes=5))

    # Simulated Drone (Drone C) - 2D mission, path crosses but at different time
    waypoints_c = [
        Waypoint(x=50, y=-20, time=current_datetime + datetime.timedelta(minutes=10)), # Starts later
        Waypoint(x=50, y=70, time=current_datetime + datetime.timedelta(minutes=15))
    ]
    mission_c = Mission("DroneC", waypoints_c, current_datetime + datetime.timedelta(minutes=10), current_datetime + datetime.timedelta(minutes=15))

    return mission_a, [mission_b, mission_c]

def create_scenario_2_direct_conflict() -> tuple[Mission, list[Mission]]:
    """Scenario: Drones flying directly towards each other, guaranteed conflict."""
    current_datetime = datetime.datetime.now()

    # Primary Drone (Drone A)
    waypoints_a = [
        Waypoint(x=0, y=0, time=current_datetime),
        Waypoint(x=100, y=100, time=current_datetime + datetime.timedelta(minutes=5))
    ]
    mission_a = Mission("DroneA_Conflict", waypoints_a, current_datetime, current_datetime + datetime.timedelta(minutes=5))

    # Simulated Drone (Drone B) - Flies towards Drone A
    waypoints_b = [
        Waypoint(x=100, y=0, time=current_datetime),
        Waypoint(x=0, y=100, time=current_datetime + datetime.timedelta(minutes=5))
    ]
    mission_b = Mission("DroneB_Conflict", waypoints_b, current_datetime, current_datetime + datetime.timedelta(minutes=5))

    # Another drone far away
    waypoints_c = [
        Waypoint(x=-50, y=-50, time=current_datetime),
        Waypoint(x=-60, y=-60, time=current_datetime + datetime.timedelta(minutes=2))
    ]
    mission_c = Mission("DroneC_Safe", waypoints_c, current_datetime, current_datetime + datetime.timedelta(minutes=2))

    return mission_a, [mission_b, mission_c]

def create_scenario_3_3d_conflict() -> tuple[Mission, list[Mission]]:
    """Scenario: Drones having 3D conflict."""
    current_datetime = datetime.datetime.now()

    # Primary Drone (Drone A) - ascending flight
    waypoints_a = [
        Waypoint(x=0, y=0, z=0, time=current_datetime),
        Waypoint(x=50, y=50, z=100, time=current_datetime + datetime.timedelta(minutes=2)),
        Waypoint(x=100, y=100, z=100, time=current_datetime + datetime.timedelta(minutes=4))
    ]
    mission_a = Mission("DroneA_3D", waypoints_a, current_datetime, current_datetime + datetime.timedelta(minutes=4))

    # Simulated Drone (Drone B) - crossing path at same altitude then ascending
    waypoints_b = [
        Waypoint(x=100, y=0, z=50, time=current_datetime + datetime.timedelta(minutes=1)),
        Waypoint(x=0, y=100, z=50, time=current_datetime + datetime.timedelta(minutes=3))
    ]
    mission_b = Mission("DroneB_3D", waypoints_b, current_datetime + datetime.timedelta(minutes=1), current_datetime + datetime.timedelta(minutes=3))

    # Simulated Drone (Drone C) - different altitude
    waypoints_c = [
        Waypoint(x=0, y=0, z=200, time=current_datetime),
        Waypoint(x=100, y=100, z=200, time=current_datetime + datetime.timedelta(minutes=4))
    ]
    mission_c = Mission("DroneC_3DSafe", waypoints_c, current_datetime, current_datetime + datetime.timedelta(minutes=4))

    return mission_a, [mission_b, mission_c]

def run_scenario(scenario_name: str, primary_mission: Mission, simulated_missions: List[Mission], safety_buffer: float):
    """Runs a deconfliction scenario and generates visualizations."""
    print(f"\n--- Running Scenario: {scenario_name} ---")
    deconflict_system = DeconflictionSystem(simulated_missions, safety_buffer=safety_buffer)
    
    result = deconflict_system.verify_mission(primary_mission)
    
    print(f"Verification Status: {result['status']}")
    if result['conflict_details']:
        print("Conflict Details:")
        for conflict in result['conflict_details']:
            print(f"  - {conflict}")
    else:
        print("No conflicts detected.")

    # Generate visualizations
    print("Generating 2D plot...")
    plot_trajectories_2d(primary_mission, simulated_missions, result['conflict_details'], safety_buffer, f"output/{scenario_name}_2D.png")
    
    print("Generating 3D plot...")
    plot_trajectories_3d(primary_mission, simulated_missions, result['conflict_details'], safety_buffer, f"output/{scenario_name}_3D.png")

    print("Generating 4D animation (this might take a while)...")
    animate_trajectories_4d(primary_mission, simulated_missions, result['conflict_details'], safety_buffer, f"output/{scenario_name}_4D_animation.gif")
    print(f"Visualizations saved to 'output/{scenario_name}_*.png' and 'output/{scenario_name}_4D_animation.gif'")


if __name__ == "__main__":
    output_dir = "output"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    SAFETY_BUFFER_DISTANCE = 10.0 # meters

    # Scenario 1: Conflict-Free
    primary_mission_1, simulated_missions_1 = create_scenario_1_conflict_free()
    run_scenario("Scenario1_ConflictFree", primary_mission_1, simulated_missions_1, SAFETY_BUFFER_DISTANCE)

    # Scenario 2: Direct Conflict (2D)
    primary_mission_2, simulated_missions_2 = create_scenario_2_direct_conflict()
    run_scenario("Scenario2_DirectConflict", primary_mission_2, simulated_missions_2, SAFETY_BUFFER_DISTANCE)

    # Scenario 3: 3D Conflict
    primary_mission_3, simulated_missions_3 = create_scenario_3_3d_conflict()
    run_scenario("Scenario3_3DConflict", primary_mission_3, simulated_missions_3, SAFETY_BUFFER_DISTANCE)

    print("\n--- All scenarios completed ---")