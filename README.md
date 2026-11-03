# TurtleBot3 Visual Navigation & Docking in MuJoCo

Welcome to **Group 80's** repository for the Integrated Project for **Track A** in *41014 Sensors and Control for Mechatronic Systems*.

An end-to-end autonomous navigation, state estimation, and visual docking pipeline for a TurtleBot3 Waffle Pi simulated in MuJoCo. Built for Track A.

The robot safely navigates a structured room, avoids obstacles using 360-degree LiDAR ray casting, tracks its position via an Extended Kalman Filter (EKF) operating on estimated states, and docks precisely onto an ArUco-marked charging station.

---

## Project & Group Details

* **Course:** 41014 Sensors and Control for Mechatronic Systems
* **Track:** Track A - Mobile Robot Visual Waypoint Navigation
* **Group Number:** Group 80
* **Platform:** TurtleBot3 Waffle Pi (Simulated in MuJoCo)

## Individual Contributions & Statement

* **Juan Coquis Morales (25807761):** Designed the MuJoCo simulation environment, built the custom room track layout (`simRoom.xml`), and managed version control. Responsibilities included developing the ArUco marker generator (`aruco.py`), the LiDAR ray-casting sensor model (`lidar.py`), and co-developing the main simulation loop (`sim.py).

* **Derek Chua:** Responsible for developing the Extended Kalman Filter state estimator (`estimator.py`) with sensor noise, built the waypoint tracking and LiDAR obstacle avoidance controller (`controller.py`), co-developed the main simulation loop (`sim.py`), and defined the platform kinematic model (`model.py`).

---

## Complete File Breakdown and Purpose

Below is a detailed list of every file in the repository, explaining what it is and its specific purpose within the projects architecture:

* **`sim.py`:** The main execution script. Its purpose is to run the real-time MuJoCo physics loop, process camera frames for ArUco decoding, update the EKF state estimation, and manage control switching between waypoint navigation, obstacle avoidance, and visual docking.

* **`controller.py`:** The navigation and control script. Its purpose is to calculate the unicycle proportional steering commands (`compute_control_command`) to reach target waypoints, and manage LiDAR avoidance state machine (`compute_obstacle_avoidance_command`) to safely steer around obstacles.

* **`estimator.py`:** The state estimation script. Its purpose is to implement an Extended Kalman Filter (EKF) that integrates the wheel encoder odometry predictions with real-time camera measurements (range and bearing to ArUco Marker 7) to mitigate drift.

* **`model.py`:** The kinematic modelling script. Its purpose is to define the non-linear unicycle equations of motion and evaluate system controllability and observability via matrix analysis.

* **`lidar.py`:** The sensor simulation script. Its purpose is to cast virtual laser rays through the MuJoCo physics engine to simulate 360-degree LiDAR scanner providing clean distance profiles.

* **`aruco.py`:** The marker generator script. Its puropse is to create and export the visual ArUco Marker 7 texture asset used on the charging station.

* **`simRoom.xml`:** The custom room simulation scene file. Its purpose is to define the 3D environment layout in MuJoCo, including room boundaries, internal obstacles (`Block1`, `Block2`, `Block3`), and charging station.

* **`turtlebot3_waffle_pi.xml`:** The core robot model file. Its purpose is to configure the Waffle Pi physical properties, wheel joints, velocity actuators, LiDAR sensor site, and front camera specifications.

* **`robotis_tb3/assets/`:** The asset folder. Its purpose is to store 3D mesh files (`.stl`) and textures (`.jpg` / `.png`) required for rendering physical geometries in MuJoCo.

---

## Rubric Alignment & Criteria Breakdown (Parts A-E)

* **Part A: Modelling (`model.py`):** We derived the non-linear unicycle kinematics model for the TurtleBot3 and performed linearisation and rank checks on the system matrices to verify that the system is fully controllable and observable.

* **Part B & C: Sensing & Estimation (`estimator.py`, `aruco.py`, `lidar.py`):** Structured coordinate transformations map local sensor measurements into global positions and project 3D points onto the 2D image plane. The EKF deliberately balances process noise and camera measurement covariance to maintain filter consistency and handle brief line-of-sight occlusions.

* **Part D: Control & Obstacle Avoidance (`controller.py`):** Waypoint tracking uses proportional error laws bounded by the Waffle Pi hardware constraints. When the front LiDAR cone detects an obstacle within a threshold limit, the robot executes the avoidance turn and uses side-indices LiDAR tracking to safely move around the barrier until the route clears.

* **Part E: Integration & Robustness (`sim.py`):** The pipeline executes end-to-end seamlessly in MuJoCo. Upon clearing waypoints, the system transitions into a visual docking loop, aligning with and stopping withint the ArUco Marker 7.

---

## Installation & Execution Instructions

Follow these terminal commands to install Python, create a virtual environment, install dependencies, and run the simulation

### Step 1: Install Python

Ensure Python (version 3.8 or higher) is installed on your system from [python.org](https://www.python.org/downloads/).

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
```

## Generative AI Declaration

* **Tools Used:** ChatGPT (OpenAI), Gemini (Google AI)
* **Purpose:** Utilised for debugging Python syntax, structuring state-machine avoidance logic, and organising repository documentation.
