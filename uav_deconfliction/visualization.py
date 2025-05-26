import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import numpy as np
import datetime
from typing import List, Tuple, Optional
from uav_deconfliction.data_models import Waypoint, Mission, Conflict
from uav_deconfliction.trajectory_generation import interpolate_trajectory, get_position_at_time # <--- ADD get_position_at_time HERE

# from typing import List, Tuple, Optional
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.animation import FuncAnimation
# import numpy as np
# import datetime

# from uav_deconfliction.data_models import Waypoint, Mission, Conflict
# from uav_deconfliction.trajectory_generation import interpolate_trajectory

def plot_trajectories_2d(
    primary_mission: Mission,
    simulated_missions: List[Mission],
    conflicts: Optional[List[Conflict]] = None,
    safety_buffer: float = 0,
    filename: str = "2d_trajectories.png",
    show_plot: bool = False
):
    """Plots 2D trajectories of drones and highlights conflicts."""
    plt.figure(figsize=(10, 8))
    ax = plt.gca()

    # Plot primary drone
    primary_trajectory = interpolate_trajectory(primary_mission)
    px = [wp.x for wp in primary_trajectory]
    py = [wp.y for wp in primary_trajectory]
    plt.plot(px, py, 'b-', label=f'Primary Drone ({primary_mission.drone_id})', linewidth=2)
    plt.plot(primary_mission.waypoints[0].x, primary_mission.waypoints[0].y, 'bo', markersize=8) # Start
    plt.plot(primary_mission.waypoints[-1].x, primary_mission.waypoints[-1].y, 'bx', markersize=8) # End

    # Plot simulated drones
    colors = plt.cm.get_cmap('Dark2', len(simulated_missions))
    for i, sim_mission in enumerate(simulated_missions):
        sim_trajectory = interpolate_trajectory(sim_mission)
        sx = [wp.x for wp in sim_trajectory]
        sy = [wp.y for wp in sim_trajectory]
        plt.plot(sx, sy, color=colors(i), linestyle='--', label=f'Simulated Drone ({sim_mission.drone_id})')
        plt.plot(sim_mission.waypoints[0].x, sim_mission.waypoints[0].y, marker='o', color=colors(i), markersize=6)
        plt.plot(sim_mission.waypoints[-1].x, sim_mission.waypoints[-1].y, marker='x', color=colors(i), markersize=6)

    # Highlight conflicts
    if conflicts:
        conflict_x = [(c.location.x + c.location.x) / 2 for c in conflicts] # Midpoint for visualization
        conflict_y = [(c.location.y + c.location.y) / 2 for c in conflicts]
        plt.scatter(conflict_x, conflict_y, color='red', marker='X', s=200, zorder=5, label='Conflict Location')
        if safety_buffer > 0:
            for cx, cy in zip(conflict_x, conflict_y):
                circle = plt.Circle((cx, cy), safety_buffer, color='red', fill=False, linestyle=':', linewidth=1)
                ax.add_patch(circle)

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('2D Drone Trajectories and Conflicts')
    plt.grid(True)
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box') # Keep aspect ratio for spatial accuracy
    plt.savefig(filename)
    if show_plot:
        plt.show()
    plt.close()

def plot_trajectories_3d(
    primary_mission: Mission,
    simulated_missions: List[Mission],
    conflicts: Optional[List[Conflict]] = None,
    safety_buffer: float = 0,
    filename: str = "3d_trajectories.png",
    show_plot: bool = False
):
    """Plots 3D trajectories of drones and highlights conflicts."""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Plot primary drone
    primary_trajectory = interpolate_trajectory(primary_mission)
    px = [wp.x for wp in primary_trajectory]
    py = [wp.y for wp in primary_trajectory]
    pz = [wp.z for wp in primary_trajectory]
    ax.plot(px, py, pz, 'b-', label=f'Primary Drone ({primary_mission.drone_id})', linewidth=2)
    ax.scatter(primary_mission.waypoints[0].x, primary_mission.waypoints[0].y, primary_mission.waypoints[0].z,
               color='blue', marker='o', s=80, label='Start') # Start
    ax.scatter(primary_mission.waypoints[-1].x, primary_mission.waypoints[-1].y, primary_mission.waypoints[-1].z,
               color='blue', marker='x', s=80, label='End') # End


    # Plot simulated drones
    colors = plt.cm.get_cmap('Dark2', len(simulated_missions))
    for i, sim_mission in enumerate(simulated_missions):
        sim_trajectory = interpolate_trajectory(sim_mission)
        sx = [wp.x for wp in sim_trajectory]
        sy = [wp.y for wp in sim_trajectory]
        sz = [wp.z for wp in sim_trajectory]
        ax.plot(sx, sy, sz, color=colors(i), linestyle='--', label=f'Simulated Drone ({sim_mission.drone_id})')
        ax.scatter(sim_mission.waypoints[0].x, sim_mission.waypoints[0].y, sim_mission.waypoints[0].z,
                   color=colors(i), marker='o', s=60) # Start
        ax.scatter(sim_mission.waypoints[-1].x, sim_mission.waypoints[-1].y, sim_mission.waypoints[-1].z,
                   color=colors(i), marker='x', s=60) # End

    # Highlight conflicts
    if conflicts:
        conflict_x = [c.location.x for c in conflicts]
        conflict_y = [c.location.y for c in conflicts]
        conflict_z = [c.location.z for c in conflicts]
        ax.scatter(conflict_x, conflict_y, conflict_z, color='red', marker='X', s=300, zorder=5, label='Conflict Location')
        if safety_buffer > 0:
            # Drawing 3D spheres for safety buffer is complex with matplotlib
            # For visualization, we can just show the conflict point itself prominently
            pass # Skipping drawing spheres for simplicity in 3D

    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_zlabel('Z Coordinate (Altitude)')
    ax.set_title('3D Drone Trajectories and Conflicts')
    plt.legend()
    plt.tight_layout()
    plt.savefig(filename)
    if show_plot:
        plt.show()
    plt.close()


def animate_trajectories_4d(
    primary_mission: Mission,
    simulated_missions: List[Mission],
    conflicts: Optional[List[Conflict]] = None,
    safety_buffer: float = 0,
    filename: str = "4d_animation.gif",
    interval_ms: int = 100, # Milliseconds between frames
    frame_time_step_seconds: float = 5.0, # How many seconds of mission time each frame advances
    show_plot: bool = False # <--- ADD THIS LINE
):
    """
    Animates 3D trajectories over time (4D visualization).
    Highlights drone positions at each time step and conflict points.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('4D Drone Trajectories (Space + Time)')

    all_missions = [primary_mission] + simulated_missions
    
    # Determine the overall time window for animation
    min_time = min(m.start_time for m in all_missions)
    max_time = max(m.end_time for m in all_missions)
    
    # Determine overall spatial bounds for consistent plot limits
    all_x, all_y, all_z = [], [], []
    for mission in all_missions:
        for wp in mission.waypoints:
            all_x.append(wp.x)
            all_y.append(wp.y)
            all_z.append(wp.z)
    ax.set_xlim([min(all_x)-10, max(all_x)+10])
    ax.set_ylim([min(all_y)-10, max(all_y)+10])
    ax.set_zlim([min(all_z)-10 if min(all_z) < 0 else 0, max(all_z)+10])


    # Prepare lists to hold line objects and point objects for animation
    lines = [] # For the full paths
    points = [] # For the current positions of drones
    conflict_markers = [] # For showing active conflicts

    # Pre-calculate full trajectories for plotting as static background lines
    pre_interpolated_trajectories = {}
    for mission in all_missions:
        traj = interpolate_trajectory(mission, time_step_seconds=10.0) # Lower resolution for static path
        pre_interpolated_trajectories[mission.drone_id] = traj
        x_coords = [wp.x for wp in traj]
        y_coords = [wp.y for wp in traj]
        z_coords = [wp.z for wp in traj]
        color = 'b' if mission == primary_mission else plt.cm.get_cmap('Dark2')(simulated_missions.index(mission))
        line, = ax.plot(x_coords, y_coords, z_coords, linestyle=':', color=color, alpha=0.5, label=f'Path {mission.drone_id}')
        lines.append(line)

    # Initialize drone positions
    for i, mission in enumerate(all_missions):
        color = 'b' if mission == primary_mission else plt.cm.get_cmap('Dark2')(simulated_missions.index(mission))
        point, = ax.plot([], [], [], 'o', color=color, markersize=8, label=f'Current {mission.drone_id}')
        points.append(point)

    # Add a text label for current time
    time_text = ax.text2D(0.05, 0.95, '', transform=ax.transAxes)

    def update(frame):
        current_time = min_time + datetime.timedelta(seconds=frame * frame_time_step_seconds)

        # Clear previous conflict markers
        for marker in conflict_markers:
            marker.remove()
        conflict_markers.clear()

        # Update drone positions
        for i, mission in enumerate(all_missions):
            pos = get_position_at_time(mission, current_time)
            if pos:
                points[i].set_data([pos.x], [pos.y])
                points[i].set_3d_properties([pos.z])
            else:
                # Hide if drone is not active
                points[i].set_data([], [])
                points[i].set_3d_properties([])
        
        # Check and display active conflicts for this frame
        if conflicts:
            for conflict in conflicts:
                # If the current frame time is very close to the conflict time
                if abs((conflict.time - current_time).total_seconds()) < frame_time_step_seconds / 2:
                    
                    # Highlight the actual drone positions involved at conflict time
                    # It's better to show the individual drone points at the conflict time
                    # rather than just the averaged conflict_location
                    
                    # Find positions of conflicting drones at this exact conflict time
                    d1_pos = get_position_at_time(primary_mission if conflict.conflicting_drone_ids[0] == primary_mission.drone_id else next(m for m in simulated_missions if m.drone_id == conflict.conflicting_drone_ids[0]), conflict.time)
                    d2_pos = get_position_at_time(primary_mission if conflict.conflicting_drone_ids[1] == primary_mission.drone_id else next(m for m in simulated_missions if m.drone_id == conflict.conflicting_drone_ids[1]), conflict.time)

                    if d1_pos and d2_pos:
                        # Draw a sphere around the average conflict location
                        avg_x = (d1_pos.x + d2_pos.x) / 2
                        avg_y = (d1_pos.y + d2_pos.y) / 2
                        avg_z = (d1_pos.z + d2_pos.z) / 2
                        
                        # Add a large red marker at the average conflict location
                        marker_point, = ax.plot([avg_x], [avg_y], [avg_z], 'X', color='red', markersize=15, markeredgewidth=2, zorder=10)
                        conflict_markers.append(marker_point)
                        
                        # Optionally, draw a line between the two drones if they are close
                        # plot_line, = ax.plot([d1_pos.x, d2_pos.x], [d1_pos.y, d2_pos.y], [d1_pos.z, d2_pos.z], 'r--', linewidth=1)
                        # conflict_markers.append(plot_line)


        time_text.set_text(f'Time: {current_time.strftime("%H:%M:%S")}')
        
        return lines + points + conflict_markers + [time_text]

    # Calculate total frames needed
    total_duration_seconds = (max_time - min_time).total_seconds()
    num_frames = int(total_duration_seconds / frame_time_step_seconds) + 2 # +2 for start and end buffer

    ani = FuncAnimation(fig, update, frames=num_frames, blit=False, interval=interval_ms, repeat=False)
    
    # Save the animation
    ani.save(filename, writer='pillow' if filename.endswith('.gif') else 'ffmpeg')
    if show_plot:
        plt.show()
    plt.close()