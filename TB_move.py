import time 
import mujoco
import mujoco.viewer
from pathlib import Path

#Path to Turtlebot3 scene
model_path = (Path(__file__).parent / "robotis_tb3" / "scene_turtlebot3_waffle_pi.xml")

# these lines load the mujoCo creating the sim data
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

# find the two tb wheel actuators
left_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_left")
right_actuator_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_right")

left_wheel_speed = 4.0
right_wheel_speed = 2.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # settting wheel speeds 
        data.ctrl[left_actuator_id] = left_wheel_speed
        data.ctrl[right_actuator_id] = right_wheel_speed

        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = model.opt.timestep - (
            time.time() - step_start)

        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


# So far this code has been Adapted from 
# rFalque/mujoco_UR3_tutorial, simulate_ur3e.py and course provided demo
# the same code is used but i just changed the path for Turtlebot
