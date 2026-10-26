import time 
import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np

# Used lab 2 functions 
# Update TB pose using the wheel position
def update_odometry(x, dphi_L, dphi_R, r, W): 
    #tb total forward distance 
    d = 0.5 * r * (dphi_L  + dphi_R)
    #tbs change in heading  
    dpsi = r * (dphi_R - dphi_L) / W
    return x + np.array([ # Add the movement to the last pose 
        d * np.cos(x[2] + 0.5 * dpsi), # add the change in world x
        d * np.sin(x[2] + 0.5 * dpsi), #changing the world y 
        dpsi, #changes the tb heading
    ])


#Path to Turtlebot3 scene
model_path = (Path(__file__).parent / "robotis_tb3" / "scene_turtlebot3_waffle_pi.xml")

# these lines load the mujoCo creating the sim data
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

wheel_radius = 0.033 
wheel_separation = 0.288
odom = np.zeros(3) # creates the array of zeros setting x, y and heading to 0
previous_left = float(data.joint("wheel_left").qpos[0])# stores tbs initial left whee; angle rads
previous_right = float(data.joint("wheel_right").qpos[0]) # stores TBs intitial right wheel angle in rads

# find the two tb wheel actuators
left_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_left")
right_actuator_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_right")

left_wheel_speed = 4.0
right_wheel_speed = 2.0

next_print_time = 1.0 # only add the first display after 1 sec will use this in the loop

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # settting wheel speeds 
        data.ctrl[left_actuator_id] = left_wheel_speed
        data.ctrl[right_actuator_id] = right_wheel_speed

        mujoco.mj_step(model, data)

        left_angle = float(data.joint("wheel_left").qpos[0]) # read the actual stored angle
        right_angle = float(data.joint("wheel_right").qpos[0]) # read the actual stored angle

        dphi_L = left_angle - previous_left # finds the left wheels rotation from its previous step
        dphi_R = right_angle - previous_right # finds the left wheels rotation from its previous step 

        previous_left = left_angle # saves left angle 
        previous_right = right_angle # saves right angle

        odom = update_odometry(odom, dphi_L, dphi_R, wheel_radius, wheel_separation)

        if data.time >= next_print_time: # display this print once for every second in sim
            print(f"t={data.time:.1f}s |" f"x={odom[0]:.3f}m | "
                   f"y={odom[1]:.3f}m | " f"heading={np.degrees(odom[2]):.1f}deg")
            #simulation time, estimated x and y and heading in deg

            next_print_time += 1.0 #schedule next 


        viewer.sync() # updating the viewer

        time_until_next_step = model.opt.timestep - (
            time.time() - step_start)

        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


# So far this code has been Adapted from 
# rFalque/mujoco_UR3_tutorial, simulate_ur3e.py and course provided demo
# the same code is used but i just changed the path for Turtlebot
