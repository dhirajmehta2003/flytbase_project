# UAV Deconfliction System: Project Plan and Design Document

## 1. Introduction

This document outlines the project plan and architectural design for an Automated UAV Trajectory Deconfliction System. The primary goal of this system is to enhance aviation safety by proactively identifying potential mid-air collisions between Unmanned Aerial Vehicles (UAVs) based on their pre-defined flight plans. By simulating trajectories and performing spatial and temporal proximity checks, the system aims to alert operators to conflicts before they occur, facilitating timely intervention and re-planning.

### 1.1 Purpose
The system serves to:
* Process and manage UAV mission data.
* Accurately interpolate drone trajectories over time.
* Detect spatial proximity breaches within a defined safety buffer.
* Verify temporal overlap at points of spatial proximity.
* Provide clear conflict notifications.
* Visualize drone flight paths and identified conflict points.

### 1.2 Scope
The current version focuses on:
* Deconfliction of pre-planned, fixed-waypoint trajectories.
* Detection of conflicts between two or more UAVs.
* Reporting of conflict locations and times.
* Basic 2D, 3D, and 4D visualization of scenarios.

## 2. Requirements

### 2.1 Functional Requirements

* **FR1: Mission Definition Input:** The system shall accept UAV mission plans defined by a sequence of waypoints, each including X, Y, Z coordinates and an optional timestamp.
* **FR2: Trajectory Interpolation:** The system shall generate continuous trajectories between waypoints based on linear interpolation over time. If waypoint times are not provided, the system shall infer them based on mission start/end times and distances.
* **FR3: Spatial Proximity Detection:** The system shall identify all time instances where any two drone trajectories come within a user-defined safety buffer distance.
* **FR4: Temporal Overlap Verification:** For spatially proximate instances, the system shall confirm if the involved drones are simultaneously within a user-defined time proximity threshold of the conflict location.
* **FR5: Conflict Reporting:** Upon detection of a confirmed conflict, the system shall report:
    * The location (X, Y, Z coordinates) of the conflict.
    * The time of the conflict.
    * The unique IDs of all conflicting drones.
    * A descriptive message of the conflict.
* **FR6: Scenario Management:** The system shall allow definition and execution of multiple deconfliction scenarios.
* **FR7: Visualization - 2D Plotting:** The system shall generate a 2D (X-Y plane) static plot of all drone trajectories, highlighting any conflict points.
* **FR8: Visualization - 3D Plotting:** The system shall generate a 3D (X-Y-Z) static plot of all drone trajectories, highlighting any conflict points.
* **FR9: Visualization - 4D Animation:** The system shall generate a time-series animation (GIF) showing the movement of drones in 3D space, with conflicts highlighted dynamically.

### 2.2 Non-Functional Requirements

* **NFR1: Performance:** Conflict detection and visualization generation should be reasonably fast for typical mission complexities (e.g., within seconds for small-to-medium numbers of drones and waypoints).
* **NFR2: Accuracy:** Trajectory interpolation and conflict detection shall be mathematically accurate within standard floating-point precision.
* **NFR3: Modularity:** The codebase shall be structured into logical modules to facilitate maintainability, understanding, and future expansion.
* **NFR4: Extensibility:** It should be straightforward to add new deconfliction algorithms, mission types, or visualization options.
* **NFR5: Usability:** The system shall have a simple command-line interface for execution and produce clear, understandable outputs.

## 3. System Architecture

The system follows a modular architecture, organized into several Python modules, each responsible for a distinct set of functionalities.

```mermaid
graph TD
    A[main.py: Scenario Orchestration] --> B(DeconflictionSystem: Core Logic)
    B --> C(data_models.py: Data Structures)
    B --> D(trajectory_generation.py: Path Interpolation)
    B --> E(spatial_check.py: Spatial Proximity)
    B --> F(temporal_check.py: Temporal Overlap)
    B --> G(visualization.py: Plotting & Animation)
    A -- Data Flow --> C
    A -- Visualization Call --> G
    subgraph Tests
        H[tests/test_*.py: Unit & Integration Tests] --> C
        H --> D
        H --> E
        H --> F
        H --> B
    end

    