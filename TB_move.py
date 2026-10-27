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

        if data.time >= next_print_time: # compare once for every second in sim
            base_pose = data.joint("base_joint").qpos.copy() # find the tb true base position and orientation

            true_x = float(base_pose[0]) # true world x 
            true_y = float(base_pose[1]) # true world y 
            qw, qx, qy, qz = base_pose[3:7] # read the orientation quaternion w, x, y, z order

            true_head = np.arctan2(2 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
            #Convert Tbs 3D orientation into its heading angle in rads

            #claculate the distance between true and estimated turtlebot positions.
            position_error = np.hypot(odom[0] - true_x, odom[1] - true_y)

            heading_error = odom[2] - true_head # calc estimated - true head
            #this heading error will be shown as the shortest angle between both directions
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

            print(f"t={data.time:.1f}s | " f"odom=({odom[0]:.3f}, {odom[1]:.3f}) | " 
                  f"pos error={position_error:.4f}m | " f"head error={np.degrees(heading_error):.2f}deg")
            # print sim time, estimated positions, true positions, pos error, heading error

            next_print_time += 1.0 #schedule next 


        viewer.sync() # updating the viewer

        time_until_next_step = model.opt.timestep - (
            time.time() - step_start)

        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


# So far this code has been Adapted from 
# rFalque/mujoco_UR3_tutorial, simulate_ur3e.py and course provided demo
# the same code is used but i just changed the path for Turtlebot
