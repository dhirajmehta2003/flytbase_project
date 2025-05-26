# UAV Deconfliction System

## Project Title: Automated UAV Trajectory Deconfliction System

## Short Description
This project implements an automated system for detecting potential mid-air conflicts between Unmanned Aerial Vehicle (UAV) trajectories. It processes pre-defined mission plans for multiple drones, interpolates their flight paths, and performs both spatial and temporal checks to identify instances where drones violate a specified safety buffer. The system provides clear status outputs and generates visualizations (2D plots, 3D plots, and 4D animations) to illustrate trajectories and highlight detected conflict points.

## Features
* **Mission Definition:** Define drone missions with sequences of waypoints including spatial coordinates (X, Y, Z) and optional timestamps.
* **Trajectory Interpolation:** Generate detailed flight trajectories from mission waypoints, linearly interpolating positions over time.
* **Spatial Proximity Check:** Efficiently identify potential conflicts by checking if any two drone trajectories come within a defined safety distance.
* **Temporal Overlap Check:** For spatially proximate points, verify if drones are simultaneously at the conflicting location within a specified time window.
* **Conflict Reporting:** Output clear status messages for each scenario (clear or conflict detected) and detailed conflict information (location, time, involved drones).
* **Comprehensive Visualization:**
    * **2D Plots:** Top-down view of drone trajectories and conflict points.
    * **3D Plots:** Isometric view of trajectories and conflicts in 3D space.
    * **4D Animations:** Time-series GIFs showing drone movement and highlighting conflicts dynamically.
* **Modular Design:** Separated concerns for data models, trajectory generation, spatial/temporal checks, and visualization, facilitating easy understanding and extension.
* **Unit Testing:** Robust test suite covering individual components to ensure reliability and correctness.

## Installation

### Prerequisites
* Python 3.8+ (Recommended Python 3.9 or higher for optimal performance with modern libraries)
* `pip` (Python package installer)

### Project Setup
1.  **Clone the Repository:**
    Ensure your project structure looks like this:
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
│       ├── test_data_models.py
│       ├── test_deconfliction_system.py
│       ├── test_spatial_check.py
│       ├── test_temporal_check.py
│       └── test_trajectory_generation.py
├── README.md                       # High-level project overview and quick start
├── PROJECT_PLAN_AND_DESIGN.md      # Detailed requirements and architectural design
├── USER_GUIDE.md                   # Instructions for end-users on how to run and interpret results
└── DEVELOPER_GUIDE.md              # Guide for understanding and extending the code

2.  **Install Dependencies:**
    Navigate to your `FlytBase_project` directory in the terminal/command prompt.
    ```bash
    cd path/to/your/FlytBase_project
    ```
    Then install the required Python libraries using the `requirements.txt` file:
    ```bash
    pip install -r uav_deconfliction/requirements.txt
    ```

## How to Run the System

To execute the deconfliction system and generate scenario results and visualizations:

1.  **Navigate to the project root directory:**
    Ensure your terminal is in the directory containing the `uav_deconfliction` folder (e.g., `C:path/to/your/FlytBase_project`).

2.  **Run the main script:**
    ```bash
    python -m uav_deconfliction.main
    ```
    This will run the pre-defined scenarios, print results to the console, and generate plots and animations in an `output/` directory within `uav_deconfliction/`.

## How to Run Tests

To verify the functionality of individual components and the overall system:

1.  **Navigate to the project root directory:**
    Ensure your terminal is in the directory containing the `uav_deconfliction` folder.

2.  **Run the test suite:**
    ```bash
    python -m unittest discover uav_deconfliction/tests
    ```
    This command will discover and execute all unit tests, providing feedback on their success or failure.

## Project Structure
