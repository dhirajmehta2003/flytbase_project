# UAV Deconfliction System: User Guide

This guide will help you get started with the UAV Deconfliction System, run the pre-defined scenarios, and understand the output generated.

## 1. Getting Started

Before you run the system, ensure you have completed the installation steps outlined in the `README.md` file in the project's root directory.

In summary, you should have:
* **Python 3.8+** installed.
* All required libraries installed by running `pip install -r uav_deconfliction/requirements.txt` from your project's root directory.
* Your project structure correctly set up as described in the `README.md`.

## 2. Running the Deconfliction System

The core of the deconfliction system is executed via the `main.py` script. This script contains several pre-defined scenarios that demonstrate various conflict (and no-conflict) situations.

### 2.1 How to Run

1.  **Open your terminal or command prompt.**
2.  **Navigate to your project's root directory.** This is the folder that *contains* the `uav_deconfliction` folder.
    For example, if your `uav_deconfliction` folder is inside `C:path/to/your/FlytBase_project`, you would navigate to `C:path/to/your/FlytBase_project`.
    ```bash
    Cd path/to/your/FlytBase_project
    ```
3.  **Execute the `main.py` script as a Python module:**
    ```bash
    python -m uav_deconfliction.main
    ```

### 2.2 What to Expect During Execution

* The script will print progress messages to your console, indicating which scenario is currently being processed (e.g., "--- Running Scenario: Scenario1_ConflictFree ---").
* For each scenario, it will report the "Verification Status" (e.g., "clear" or "conflict detected") and any conflict details if found.
* You'll see messages indicating the generation of 2D, 3D, and 4D plots/animations.
* The 4D animation generation can take some time, especially the first time it runs, as Matplotlib might build a font cache. Please be patient.

## 3. Understanding the Pre-defined Scenarios

The `main.py` file includes several scenarios designed to test different aspects of the deconfliction system:

* **Scenario1_ConflictFree:** Two drones with paths that do not intersect within the safety buffer. Expected: No conflict.
* **Scenario2_2DConflict:** Two drones whose paths intersect in the X-Y plane within the safety buffer, and they arrive at the conflict point simultaneously. Expected: Conflict.
* **Scenario3_TimeSeparatedNoConflict:** Two drones whose paths intersect spatially, but they arrive at the conflict point at significantly different times (outside the `time_proximity_threshold`). Expected: No conflict.
* **Scenario4_3DConflict:** Two drones whose paths intersect in 3D space, and they arrive at the conflict point simultaneously. Expected: Conflict.
* **Scenario5_MultipleDronesConflict:** Three or more drones where at least two pairs experience conflicts. Expected: Multiple conflicts reported.
* **Scenario6_NearMiss:** Drones pass very close but just outside the safety buffer, or the conflict is resolved by a very minor adjustment (if deconfliction logic were dynamic). Expected: No conflict (or a very close call reported as "clear").

## 4. Interpreting the Output

After execution, the system provides both console output and visual deliverables.

### 4.1 Console Output

The console will show real-time feedback:

* **`--- Running Scenario: [Scenario Name] ---`**: Indicates the start of a new deconfliction scenario.
* **`Verification Status: clear`**: No conflicts were detected for the primary drone within the specified safety buffer.
* **`Verification Status: conflict detected`**: One or more conflicts were found involving the primary drone.
* **`Conflict detected for DRONE_A and DRONE_B at Location (X, Y, Z) at Time [HH:MM:SS] (Description: ...)`**: This detailed message appears if a conflict is found, providing crucial information about the event.
* **`Generating 2D plot...`, `Generating 3D plot...`, `Generating 4D animation...`**: Status updates during visualization generation.

### 4.2 Visual Outputs

An `output/` directory will be created inside your `uav_deconfliction` folder (`uav_deconfliction/output/`). This folder will contain the following files for each scenario:

* **`[ScenarioName]_2D_plot.png`**:
    * **Description:** A static image showing the X-Y (top-down) view of all drone trajectories.
    * **Interpretation:** Allows quick assessment of horizontal path overlaps. Conflict points (if any) are typically marked with special markers (e.g., red 'X's).
* **`[ScenarioName]_3D_plot.png`**:
    * **Description:** A static image showing the 3D (X-Y-Z) view of all drone trajectories.
    * **Interpretation:** Provides a comprehensive spatial understanding of the flight paths. Conflict points are highlighted in 3D space.
* **`[ScenarioName]_4D_animation.gif`**:
    * **Description:** An animated GIF showing the movement of each drone over time.
    * **Interpretation:** This is the most dynamic visualization. Watch the drones move, and observe how conflicts are highlighted as the drones reach the conflict location at the same time. This is invaluable for understanding the temporal aspect of conflicts.

## 5. Customizing Scenarios

If you wish to define your own drone missions or modify the existing scenarios, you can directly edit the `uav_deconfliction/main.py` file.

* Look for the `create_scenario_X` functions to see how missions and waypoints are defined.
* You can create new functions to define your missions and then add calls to `run_scenario()` at the end of `main.py` to execute them.
* Adjust `SAFETY_BUFFER_DISTANCE` and `TIME_PROXIMITY_THRESHOLD` at the top of `main.py` to change the conflict detection sensitivity.

---
