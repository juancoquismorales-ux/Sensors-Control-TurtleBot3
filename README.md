# Sensors-Control-TurtleBot3
Sensors and Control Integrated Project: Turtlebot navigation and visual docking in MujoCo
## Group number and teammates
Group project 80
- Juan Coquis Morales 25807761
- Derek Chua
- Nicholas Ricci
## 1. Group and Project Details
* **Course:** 41014 Sensors and Control for Mechatronic Systems
* **Track:** Track A - Mobile RObot Visual Waypoint Navigation
* **Platform:** TurtleBot3 Waffle Pi (in MuJoCo)

## 2. Team Members and Contributions
* **[Juan Coquis Morales]:** Created the MuJoCo simulation environment, custom room track layout (`simRoom.xml`), and managed repository organisation / Git workflow.
* **[Derek Chua]:** Developed the Extended Kalman Filter (EKF) estimator (`estimator.py`) and integraated the main simulation loop (`sim.py`).
* **[Nicholas Ricci]:** Developed the waypoint tracking controller (`controller.py`).

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
- **Tools used:** Gemini (Google AI) and ChatGPT (OpenAI) and VScode AI 
- **Purpose:** Used for debugging Python syntax, explaining complex EKF matrix equations, brainstorming control structures, and structuring project documentation for repository. Further used to explain complex lab exercises, help with debugging and cleaning code. additionally the AI in VScode helped with efficiency as it finished off long lines of code as well as helping with comment impact and meaning. It did not produce and code or comments that we werent already going to use, it would only finish lines of code we were already writting and similarly with the comments. 

