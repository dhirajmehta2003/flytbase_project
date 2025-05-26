# UAV Deconfliction System: Developer Guide

This guide provides an in-depth look at the architecture, modules, and key components of the UAV Deconfliction System. It's designed for developers who want to understand the codebase, debug issues, or extend its functionality.

---

## 1. Project Structure

The project is organized into a modular structure, where each file or directory serves a specific purpose:
.
├── uav_deconfliction/
│   ├── data_models.py              # Defines core data structures (Waypoint, Mission, Conflict)
│   ├── deconfliction_system.py     # Orchestrates the deconfliction process
│   ├── main.py                     # Entry point, defines scenarios, runs the system, and triggers visualization
│   ├── requirements.txt            # Lists all external Python dependencies
│   ├── spatial_check.py            # Handles spatial (distance) proximity checks
│   ├── temporal_check.py           # Handles temporal (time overlap) checks
│   ├── trajectory_generation.py    # Generates and interpolates drone trajectories
│   ├── visualization.py            # Manages all plotting and animation features
│   └── tests/                      # Contains all unit and integration tests
│       ├── __init__.py
│       ├── test_data_models.py
│       ├── test_deconfliction_system.py
│       ├── test_spatial_check.py
│       ├── test_temporal_check.py
│       └── test_trajectory_generation.py
├── README.md                       # High-level project overview and quick start
├── PROJECT_PLAN_AND_DESIGN.md      # Detailed requirements and architectural design
├── USER_GUIDE.md                   # Instructions for end-users on how to run and interpret results
├── DEVELOPER_GUIDE.md              # Guide for understanding and extending the code
└── REFLECTION_AND_JUSTIFICATION.md # Document discussing design, implementation, testing, and scalability
---

## 2. Core Components and Their Responsibilities

Each Python file within the `uav_deconfliction/` directory encapsulates a specific set of functionalities:

* ### `data_models.py`
    This module defines the fundamental **data structures** used throughout the system.
    * **`Waypoint` Class**: Represents a point in 3D space (`x`, `y`, `z`) at a specific `time`. It includes a `distance_to()` method for calculating Euclidean distance.
    * **`Mission` Class**: Defines a drone's flight plan, including a `drone_id`, an ordered list of `waypoints`, and the mission `start_time` and `end_time`. It contains logic to automatically assign times to waypoints if they are not explicitly provided, distributing them linearly based on the mission's duration.
    * **`Conflict` Class**: Encapsulates a detected conflict event, storing the `location` (as a `Waypoint`), the exact `time` of the conflict, the `conflicting_drone_ids`, and a `description`.

* ### `trajectory_generation.py`
    This module is responsible for **generating and interpolating drone paths**.
    * **`interpolate_trajectory(mission: Mission, time_step_seconds: int = 1) -> List[Waypoint]`**: Generates a dense list of waypoints along a mission's path. It linearly interpolates positions between defined waypoints at specified `time_step_seconds` intervals.
    * **`get_position_at_time(mission: Mission, query_time: datetime.datetime) -> Waypoint`**: Calculates the precise interpolated 3D position (X, Y, Z) of a drone along its mission at a given `query_time`. It handles cases where the `query_time` is outside the mission's defined duration by returning the start/end point.

* ### `spatial_check.py`
    This module focuses solely on **spatial proximity detection**.
    * **`calculate_distance(wp1: Waypoint, wp2: Waypoint) -> float`**: A utility function to compute the Euclidean distance between two `Waypoint` objects.
    * **`check_spatial_proximity(primary_trajectory: List[Waypoint], simulated_trajectory: List[Waypoint], safety_buffer: float) -> List[Tuple[Waypoint, Waypoint]]`**: Iterates through two interpolated trajectories to find all pairs of points (one from each trajectory) that are within the `safety_buffer` distance. It returns these as potential conflict pairs.

* ### `temporal_check.py`
    This module verifies the **temporal aspect** of potential conflicts.
    * **`check_temporal_overlap_at_location(primary_mission: Mission, simulated_mission: Mission, potential_conflict_point_primary: Waypoint, potential_conflict_point_simulated: Waypoint, safety_buffer: float, time_proximity_threshold: datetime.timedelta = datetime.timedelta(seconds=5)) -> Optional[Conflict]`**: This function takes two spatially proximate points and their respective missions. It then uses `get_position_at_time` to check if both drones are simultaneously present at the conflict location within the `time_proximity_threshold`. If confirmed, it returns a `Conflict` object.

* ### `deconfliction_system.py`
    This module is the **central orchestrator** of the deconfliction process.
    * **`DeconflictionSystem` Class**:
        * `__init__()`: Initializes the system with a `safety_buffer` and `time_proximity_threshold`.
        * `register_mission(mission: Mission)`: Adds a `Mission` to the system for deconfliction.
        * `run_deconfliction() -> Dict[str, Union[str, List[Conflict]]]` : The core method that iterates through all registered mission pairs. For each pair, it calls `interpolate_trajectory`, then `check_spatial_proximity`, and for each potential spatial conflict, it calls `check_temporal_overlap_at_location`. It aggregates and returns all detected conflicts.

* ### `visualization.py`
    This module handles all aspects of **plotting and animating** the drone trajectories and conflicts using `matplotlib`.
    * **`plot_trajectories_2d(...)`**: Generates a 2D (X-Y) static plot.
    * **`plot_trajectories_3d(...)`**: Generates a 3D (X-Y-Z) static plot.
    * **`animate_trajectories_4d(...)`**: Creates a time-series GIF animation of the drones' movement, highlighting conflicts dynamically. This function relies on `FuncAnimation` from `matplotlib.animation` and uses `get_position_at_time` to update drone positions over time.

* ### `main.py`
    This is the **application's entry point**. It sets up and runs specific scenarios.
    * Defines multiple example missions and scenarios.
    * Instantiates `DeconflictionSystem`.
    * Calls `run_deconfliction()` and then the various plotting functions from `visualization.py`.
    * It serves as a demonstration and a template for how to use the `DeconflictionSystem` and its related modules.

---

## 3. Extending the System

The modular design makes it relatively easy to extend the system's capabilities.

### 3.1 Adding New Scenarios

To add more deconfliction scenarios:
1.  **Open `uav_deconfliction/main.py`**.
2.  **Define new `Waypoint` and `Mission` objects** to describe your desired drone paths and times.
3.  **Create a new `create_scenario_X` function** (e.g., `create_scenario_7`) that returns the primary mission and a list of simulated missions for your new test case.
4.  **Call `run_scenario()`** at the end of `main.py` with your new scenario's details.

### 3.2 Modifying Deconfliction Logic

If you wish to alter how conflicts are detected:

* **Change `safety_buffer` or `time_proximity_threshold`**: These parameters are defined at the top of `main.py` and passed to `DeconflictionSystem`. Adjusting them will change the sensitivity of conflict detection.
* **Modify `spatial_check.py`**:
    * You could implement different distance metrics if needed, though Euclidean is standard.
    * You could refine the spatial sampling logic.
* **Modify `temporal_check.py`**:
    * You might introduce more complex temporal overlap conditions beyond simple time proximity (e.g., considering drone speed or intent).
* **Modify `deconfliction_system.py`**:
    * If you wanted to implement a pre-filtering step (e.g., only check drones within a certain geographical region of each other), you'd add that logic here before the pairwise comparisons.
    * You could integrate different deconfliction strategies (e.g., suggesting new waypoints) into the `run_deconfliction` loop, although this system is currently focused on detection, not resolution.

### 3.3 Adding New Visualization Types

To create new ways of visualizing trajectories or conflicts:
1.  **Open `uav_deconfliction/visualization.py`**.
2.  **Add a new function** (e.g., `plot_altitude_over_time`).
3.  This function will likely take `Mission` or `List[Waypoint]` objects as input.
4.  Use `matplotlib` or other plotting libraries to create your desired visualization.
5.  **Call your new visualization function** from `main.py` after the `run_deconfliction()` call.

### 3.4 Extending Data Models

If your project evolves to require more complex drone or mission attributes:
1.  **Open `uav_deconfliction/data_models.py`**.
2.  **Add new attributes** to `Waypoint`, `Mission`, or `Conflict` classes.
3.  **Update `__init__` methods** and any other relevant methods (e.g., `to_dict()`, `distance_to()`) to handle the new attributes.
4.  Remember to update any parts of the system (e.g., `main.py`, `trajectory_generation.py`) that create or consume these objects.

---

## 4. Testing

The `tests/` directory contains unit tests for all core modules, ensuring the correctness and reliability of the system.

### 4.1 Running Tests

To run the entire test suite:
1.  Navigate to your project's root directory.
2.  Execute: `python -m unittest discover uav_deconfliction/tests`

### 4.2 Adding New Tests

When you extend or modify the system, it's crucial to add corresponding tests:
1.  **Identify the module** you are modifying (e.g., `spatial_check.py`).
2.  **Open its corresponding test file** (e.g., `tests/test_spatial_check.py`).
3.  **Add a new method** to the relevant `unittest.TestCase` class (e.g., `TestSpatialCheck`).
4.  **Name the method starting with `test_`** (e.g., `test_new_spatial_condition`).
5.  Inside your test method, set up the necessary inputs, call the function/method you want to test, and use `self.assertEqual()`, `self.assertTrue()`, `self.assertAlmostEqual()`, etc., to assert the expected behavior.

---