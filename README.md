# TurtleBot3 Visual Navigation & Docking in MuJoCo

Welcome to **Group 80's** repository for the Integrated Project for **Track A** in *41014 Sensors and Control for Mechatronic Systems*.

An end-to-end autonomous navigation, state estimation, and visual docking pipeline for a TurtleBot3 Waffle Pi simulated in MuJoCo. Built for Track A.

The turtlebot safely navigates a structured room, avoids obstacles using 360-degree LiDAR ray casting, tracks its position via an Extended Kalman Filter (EKF) operating on estimated states, and docks precisely onto an ArUco-marked charging station.

---

## Project & Group Details

* **Course:** 41014 Sensors and Control for Mechatronic Systems
* **Track:** Track A - Mobile Robot Visual Waypoint Navigation
* **Group Number:** Group 80
* **Platform:** TurtleBot3 Waffle Pi (Simulated in MuJoCo)

## Individual Contributions & Statement

* **Juan Coquis Morales (25807761):** Designed the MuJoCo simulation environment, built the custom room track layout (`simRoom.xml`), and managed version control. Responsibilities included developing the ArUco marker generator (`aruco.py`), the LiDAR ray-casting sensor model (`lidar.py`), and co-developing the main simulation loop (`sim.py`).

* **Derek Chua:** Responsible for developing the Extended Kalman Filter state estimator (`estimator.py`) with sensor noise, built the waypoint tracking and LiDAR obstacle avoidance controller (`controller.py`), co-developed the main simulation loop (`sim.py`), and defined the platform kinematic model (`model.py`).

---

## System Architecture & Frames

### Frame Diagram
```text
[World Frame (W)] 
       │
       ▼ (Odometry / Kinematics)
[Robot Base Frame (B)] ──► [LiDAR Frame]
       │
       ▼ (Rigid Transform)
[Camera Frame (C)] ──► [ArUco Marker Frame]
```

### Key Simulation Parameters
| Parameter | Symbol / Value | Description |
| :--- | :--- | :--- |
| **Control Rate** | $50\text{ Hz}$ | Controller update frequency |
| **Target Depth** | $0.19\text{ m}$ | Estimated camera-to-marker depth target |
| **Depth Tolerance** | $\pm 0.03\text{ m}$ | Acceptable depth band around target |
| **Bearing Tolerance** | $\pm 3^\circ$ | Acceptable angular alignment tolerance |
| **EKF Bounds** | $\pm 3\sigma$ | Analytical uncertainty bounds for error tracking |

---

## Complete File Breakdown and Purpose

Below is a detailed list of every file in the repository, explaining what it is and its specific purpose within the project's architecture:

* **`sim.py`:** The main execution script. Its purpose is to run the real-time MuJoCo physics loop, process camera frames for ArUco decoding, update the EKF state estimation, and manage control switching between waypoint navigation, obstacle avoidance, and visual docking.

* **`controller.py`:** The navigation and control script. Its purpose is to calculate the unicycle proportional steering commands (`compute_control_command`) to reach target waypoints, and manage LiDAR avoidance state machine (`compute_obstacle_avoidance_command`) to safely steer around obstacles.

* **`estimator.py`:** The state estimation script. Its purpose is to implement an Extended Kalman Filter ($EKF$) that integrates the wheel encoder odometry predictions with real-time camera measurements (range and bearing to ArUco Marker 7) to mitigate drift.

* **`model.py`:** The kinematic modelling script. Its purpose is to define the non-linear unicycle equations of motion and evaluate system controllability and observability via matrix analysis.

* **`lidar.py`:** The sensor simulation script. Its purpose is to cast virtual laser rays through the MuJoCo physics engine to simulate 360-degree LiDAR scanner providing clean distance profiles.

* **`aruco.py`:** The marker generator script. Its purpose is to create and export the visual ArUco Marker 7 texture asset used on the charging station.

* **`simRoom.xml`:** The custom room simulation scene file. Its purpose is to define the 3D environment layout in MuJoCo, including room boundaries, internal obstacles (`Block1`, `Block2`, `Block3`), and charging station.

* **`turtlebot3_waffle_pi.xml`:** The core turtlebot model file. Its purpose is to configure the Waffle Pi physical properties, wheel joints, velocity actuators, LiDAR sensor site, and front camera specifications.

* **`trajectory_comparison.png` & `ekf_error_bounds.png`:** The evaluation and performance graphs. Its purpose is to provide quantitative evidence of the filter consistency and trajectory tracking by comparing EKF estimates against simulator ground truth.

* **`robotis_tb3/assets/`:** The asset folder. Its purpose is to store 3D mesh files (`.stl`) and textures (`.jpg` / `.png`) required for rendering physical geometries in MuJoCo.

---

## Evaluation Graphs & Quantitative Output

Upon closing the simulation window, `sim.py` automatically processes the logged telemetry data and saves two high-resolution performance graphs directly into the repository workspace. Satisfying the rubric's quantitative evaluation requirements.

1. **`trajectory_comparison.png` (Estimated vs Actual Path):**
    * **What it shows:** A top-down 2D room plot comparing the TurtleBot's actual trajectory from simulator ground truth (`black line`) against the EKF-estimated path (`blue dashed line`).
    * **Purpose:** Compares the estimate with the actual path. It does not measure tracking error against a desired route.

2. **`ekf_error_bounds.png`:**
    * **What it shows:** Time-series error plots for position ($X$ and $Y$) and heading ($\theta$), bounded by $\pm 3\sigma$ consistency limits (`purple dashed lines` with shaded uncertainty regions).
    * **Purpose:** Displays estimation errors plotted against $\pm \sigma$ bounds. The observed errors remain within these bounds, suggesting that the filter's uncertainty estimation appears conservative. Uncertainty reductions align with camera update intervals, while the plotting pipeline logs predictions prior to each camera correction step.

3. **`Trial evaluation`:**
    * Between each trial we varied the starting y positions in each one. Additionally in trials 1 and 3 a black wall was removed illustrating that TurtleBot could adapt to new situations etc. Throughout all trials our controller setting did not vary so all trials portrayed the same version of our controller code.
4. **`Failiures`:** 
    * Amongst each trial run our simulation did not encounter any major failiures, however, through our graph analysis we can understand that TB experienced heading drift on multiple occasions and some weird avoidance detouring made the run uneffective. Some of these errors occurred due to wheel slip from the TB, but through our camera reading our EKF estimations were able to be rectified and improved.
5.  **`Trade-offs`:** 
     * Throughout the code production we encountered two impactful trade-offs,  TurtleBot efficiency or safe and more reliable movements, accurae estimation or  better camera-marker visibility . We decided that saftey was key in this project so our detours substantially increased time, however, our system then had much more time to respond to upcomming obstacles. Additionally, we chose to implement a better camera system as we would then have better marker 7 readings to to correct the EKF estimation errors (this only worked when marker was in camera view). 
---

## Demo Video
- https://youtu.be/TFaz4NdPcP8 

---

## Rubric Alignment & Criteria Breakdown (Parts A-E)

* **Part A: Modelling (`model.py`):** We derived the non-linear unicycle kinematics model for the TurtleBot3 and performed linearisation and rank checks on the system matrices. Controllability is checked at a moving reference. The observability test assumes direct $x/y$ measurements, whereas the actual EKF uses marker range and bearing. Hence, the test does not establish observability for the actual camera model.

* **Part B & C: Sensing & Estimation (`estimator.py`, `aruco.py`, `lidar.py`):** Structured coordinate transformations map local sensor measurements into global positions and project 3D points onto the 2D image plane. The EKF deliberately balances process noise $Q$ and camera measurement covariance $R$ to maintain filter consistency and handle brief line-of-sight occlusions.

* **Part D: Control & Obstacle Avoidance (`controller.py`):** Waypoint tracking uses proportional error laws bounded by the Waffle Pi hardware constraints. When the front LiDAR cone detects an obstacle within a threshold limit, the TurtleBot executes the avoidance turn and uses side-indices LiDAR tracking to safely move around the barrier until the route clears.

* **Part E: Integration & Robustness (`sim.py`):** The pipeline executes end-to-end seamlessly in MuJoCo. Upon clearing waypoints, the system transitions into a visual docking loop targeting an estimated camera-to-marker depth of $0.19\text{ m}$ (directed by $0.03\text{ m}$ depth and $3^\circ$ bearing tolerances). This reflects target estimator states rather than a guaranteed physical distance measurement, and final bearing alignment does not formally prove that the TurtleBot chassis is square to the wall.

---

## Installation & Execution Instructions

Tested on Windows and macOS (Python 3.15.2) environments using Python and virtual environments

### Step 1: Install Python

Ensure Python (version 3.15.2 or compatible) is installed on your system from [python.org](https://www.python.org/downloads/).

### Step 2: Clone Repository

Clone this repository to your computer using Git:

```bash
git clone https://github.com/juancoquismorales-ux/Sensors-Control-TurtleBot3.git 
cd Sensors-Control-TurtleBot3
```

### Step 3: Create & Active Python Virtual Environment

Create a clean environment named `.venv`:

```bash
python -m venv .venv
```

Activate the virtual environment:

```bash
.venv\Scripts\activate
```

### Step 4: Upgrade pip & Install required Packages

Upgrade `pip` to the latest version inside the environment

```bash
pip install --upgrade pip
```

Install MuJoCo, OpenCV, and Numpy

```bash
pip install mujoco
pip install opencv-python
pip install numpy
pip install matplotlib
```

## Generative AI Declaration

* **Tools Used:** ChatGPT (OpenAI), Gemini (Google AI)
* **Purpose:** Utilised for debugging Python syntax, structuring state-machine avoidance logic, code review when bad code was troubling our simulation, some result interpretation when results were random and unexpected, organising repository documentation and helping to edit/clean up.
