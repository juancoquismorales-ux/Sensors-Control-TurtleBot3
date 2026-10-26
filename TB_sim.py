import mujoco
import mujoco.viewer
from pathlib import Path

#Path to Turtlebot3 scene
model_path = (Path(__file__).parent / "robotis_tb3" / "scene_turtlebot3_waffle_pi.xml")

# these lines load the mujoCo creating the sim data
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()

# So far this code has been Adapted from 
# rFalque/mujoco_UR3_tutorial, simulate_ur3e.py and course provided demo
# the same code is used but i just changed the path for Turtlebot



