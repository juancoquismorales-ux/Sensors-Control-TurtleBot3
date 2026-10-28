# Sensors-Control-TurtleBot3
Sensors and Control Integrated Project: Turtlebot navigation and visual docking in MujoCo

## 1. Group and Project Details
* **Course:** 41014 Sensors and Control for Mechatronic Systems
* **Track:** Track A - Mobile RObot Visual Waypoint Navigation
* **Platform:** TurtleBot3 Waffle Pi (in MuJoCo)

## 2. Team Members and Contributions
* **[Juan Coquis Morales]:** Worked on robot mnodelling, equations, and system analysis (Controllability and Observability)
* **[Derek Chua]:** Built the Extended Kalman Filter (EKF) for state estimation ion and sensor integration
* **[Nicholas Ricci]:** Developed the waypoint tracking controller and integrated into MuJoCo simulation

## 3. How the System Works
1. **Modelling (`model.py`):** Defines how the robot moves using unicycle kinematics and checks that the system can be fully controlled and observed.
2. **Estimation (`estimator.py`):** Combines wheel encoder data with sensor noise measurements using the Extended Kalman Filter (EKF) to keep track of where the robot is.
3. **Control (`controller.py`):** Calculates smooth steering and speed commands to guide the robot step-by-step toward target waypoint.
4. **Simulation (`sim.py`):** Runs the main loop inside MuJoCo, using the model, estimator, and controller together in the custom room track environment.

## 4. File Structure
* `model.py` - Robot math and kinematics formulas.
* `estimator.py` - EKF filter for tracking position and handling sensor noise.
* `controller.py` - Waypoint navigation and steering controller.
* `sim.py` - Main python script that runs the simulation window
* `simRoom.xml` - Custom simulation room layout with track (walls, obstacles, and charging station).

## 5. Gen AI Declaration
