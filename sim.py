import time
import mujoco
import mujoco.viewer
import cv2
from pathlib import Path
import numpy as np
from model import TurtleBot3Model
from estimator import TurtleBot3EKF
from controller import TurtleBot3Controller
from lidar import read_lidar

model_path = (Path(__file__).parent / "robotis_tb3" / "simRoom.xml")

model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

cam_width, cam_height = 640, 480
renderer = mujoco.Renderer(model, height=cam_height, width=cam_width)

marker_size = 0.30 # marker width in metres
camera_id = model.camera("front_camera").id # find turtlebot camera in mujoco xml file
vertical_fov = np.deg2rad(model.cam_fovy[camera_id]) # convert fov to radians
focal_points = cam_height / (2 * np.tan(vertical_fov / 2)) # calculate focal length in pixels

# describe how the configured pinhole camera projects 3D points onto the 2D image plane
camera_matrix = np.array([[focal_points, 0, cam_width / 2],
                          [0, focal_points, cam_height / 2],
                          [0, 0, 1]]) 
distortion = np.zeros(5) # no distortion in mujoco camera


#Here we are making sure that TB starting at the south wall facing the north wall
start_state = np.array([-1.5, 0, 0]) # x and y heading
base_start = data.joint("base_joint").qpos # base position and orientation
base_start[0:2] = start_state[0:2] # setting the x and y for start pos
#set orientation using the w, x, y, z quaternion
base_start[3:7] = [ np.cos(start_state[2] / 2), #quaternion w 
                   0.0, #quaternion x
                   0.0, # quaternion y
                   np.sin(start_state[2] / 2) #quaternion z
                   ]
mujoco.mj_forward(model, data) # update sim new pose


# here we need to match the EKFs prediction interval to mujocos timestep
dt = model.opt.timestep 
track_width, wheel_radius = 0.288, 0.033

ekf = TurtleBot3EKF(dt=dt)
#Since we changed starting pose we want to initialise the estimated pose at the same start pose
ekf.x_hat = start_state.copy()
controller = TurtleBot3Controller(track_width=track_width, wheel_radius=wheel_radius)

waypoints = [
    np.array([1.8, 0.0, 0.0]),
    np.array([0.8, 0.5, 0.0]),
    np.array([-0.7, -0.6, 0.0]),
    np.array([0.0, 0.0, 0.0])    
]
"""
waypoints = [
    np.array([0.5, 0.0, 0.0]),
    np.array([0.5, -0.5, 0.0]),
    np.array([-0.3, -1.2, 0.0]),
    np.array([-1.0, -0.6, 0.0]),
    np.array([-1.0, 0.0, 0.0]),
    np.array([0.0, 0.0, 0.0])
]
"""

waypoint_idx = 0
target_waypoint = waypoints[waypoint_idx]

H_matrix = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0]
])

left_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_left")
right_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_right")

previous_left = float(data.joint("wheel_left").qpos[0])
previous_right = float(data.joint("wheel_right").qpos[0])

v_cmd, omega_cmd = 0.0, 0.0 # 0 turn rate and forward speed for starting pose check

# converting the TB foward speed and turn rates into left and right wheel speed for simplicity
left_speed, right_speed = controller.velocity_to_wheel(v_cmd, omega_cmd)
# Send the left and right wheels target angular speeds in rad/s to Mujoco
data.ctrl[left_actuator_id] = left_speed
data.ctrl[right_actuator_id] = right_speed

next_print_time = 1.0

print(f"starting mujoco ekf and controller sim")

next_scan_time = 0.0 # Take the first scan when sim starts
next_camera_update = 0.0 # Take the first camera update when sim starts
next_ui_update = 0.0
scan_ranges = None

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        
        left_angle = float(data.joint("wheel_left").qpos[0])
        right_angle = float(data.joint("wheel_right").qpos[0])
        
        dphi_L = left_angle - previous_left
        dphi_R = right_angle - previous_right
        
        previous_left = left_angle
        previous_right = right_angle

        # calculate the forward distance and heading change based on wheel rotations
        d_forward = 0.5 * wheel_radius * (dphi_L + dphi_R)
        d_heading = wheel_radius *(dphi_R - dphi_L) / track_width

        measured_v = d_forward / dt # calculate the measured forward speed in m/s
        measured_omega = d_heading / dt # calculate the measured turn rate in rad/s
        estimated_state = ekf.predict(measured_v, measured_omega) # predict the next state using the EKF

        """
        renderer.update_scene(data, camera="front_camera") # render from camera view attached to TB3
        rgb_img = renderer.render()

        display_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR) # convert to BGR for OpenCV display
        cv2.imshow("TurtleBot3 Camera View", display_img) # show camera view
        cv2.waitKey(1) # help refresh the opencv window
        
        gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray_img)
        """
        marker_position = None 
        """
        if ids is not None: #only continue if a marker is detected
            for i, marker_id in enumerate(ids.flatten()): #check id's
                if marker_id == 7: # only want marker 7
                    # calcluate marker pose using the camera matrix and distortion coefficients ( relative to camera frame)
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, distortion)
                    marker_position = tvecs[0, 0].copy() # extract camera frame x, y and z position of the marker \
                        """
        if data.time >= next_camera_update:
            renderer.update_scene(data, camera="front_camera")
            rgb_img = renderer.render()
            
            gray_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray_img)
            if ids is not None:
                print(f"found aruco IDs: {ids.flatten()}")
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == 7:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, distortion)
                        marker_position = tvecs[0, 0].copy()
            if marker_position is not None:
                    
                    forward = marker_position[2] + 0.10 # distance in front of the camera
                    left = -marker_position[0] # left is negative x in camera frame

                    measurement = np.array([np.hypot(forward, left), np.arctan2(left, forward)]) 
                    # calculate the range and bearing to the marker week 7 ekf update

                    marker_position_world = np.array([1.939, 0]) # known world position of the marker in the mujoco sim room
                    # update the EKF with the marker measurement
                    estimated_state, _ = ekf.update_with_marker(marker_position_world, measurement)     

            next_camera_update = data.time + 0.05 # schedule next camera update

        distance_to_goal = np.linalg.norm(target_waypoint[:2] - estimated_state[:2])
        if distance_to_goal < 0.1 and waypoint_idx < len(waypoints) - 1:
            waypoint_idx += 1
            target_waypoint = waypoints[waypoint_idx]
            print(f"reached waypoint. switching to target index {waypoint_idx}: {target_waypoint}")

         # validation test to make sure the TB is not moving during the first 4 seconds of the sim   
        if 1 <= data.time < 4:    
            v_cmd, omega_cmd = 1.0, 0  # move forward at 0.08 m/s for the first 4 seconds of the sim
        else:
            v, omega = controller.compute_obstacle_avoidance_command(estimated_state, target_waypoint, scan_ranges)
            v_cmd, omega_cmd = v, omega
            # v_cmd, omega_cmd = 0, 0 # stop moving after 4 seconds of the sim
            
        left_speed, right_speed = controller.velocity_to_wheel(v_cmd, omega_cmd)
        
        data.ctrl[left_actuator_id] = left_speed
        data.ctrl[right_actuator_id] = right_speed
        
        mujoco.mj_step(model, data)

        if data.time >= next_scan_time: # scan every 0.1 seconds 
            # get angles, distances and time 
            scan_angles, scan_ranges, scan_time = read_lidar(model, data)

            #schedule next scan
            next_scan_time = data.time + 0.1 

        if data.time >= next_print_time:
            base_pose = data.joint("base_joint").qpos.copy()
            true_x, true_y = float(base_pose[0]), float(base_pose[1])

            # extract true heading from sim orientation quaternion (w, x, y, z) using the formula for yaw angle
            qw, qx, qy, qz = base_pose[3:7]
            true_heading = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))

            # calculate the position error between estimated and true position
            position_error = np.hypot(estimated_state[0] - true_x, estimated_state[1] - true_y) 
            heading_error = estimated_state[2] - true_heading # calculate the heading error
            # wrap the heading error to be within -pi and pi
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error)) 
        

            sigma = np.sqrt(np.diag(ekf.P)) # calculate the standard deviation of the state estimate

            print(
                f"CHECK | true=({true_x:.3f}, {true_y:.3f})m | "
                f"position error={position_error:.3f}m | "
                f"heading error={np.rad2deg(heading_error):.2f}deg | "
                f"sigma x/y=({sigma[0]:.3f}, {sigma[1]:.3f}m) | "
                f"sigma heading={np.degrees(sigma[2]):.2f}deg"
            )

            
            print(f"t = {data.time:1f}s | est pos: ({estimated_state[0]:.3f}, {estimated_state[1]:.3f}) | target index: {waypoint_idx}")
            
            print(f"LIDAR | front= {scan_ranges[0]:.2f}m | "
                  f"left={scan_ranges[90]:.2f}m | "
                  f"back={scan_ranges[180]:.2f}m | "
                  f"right={scan_ranges[270]:.2f}m" 
                  )
            
            if marker_position is not None:
                camera_x, camera_y, camera_z = marker_position 
                camera_bearing = np.arctan2(-camera_x, camera_z) 

                print(f"Marker 7 detected at camera frame: z={camera_z:.3f}m, bearing={np.rad2deg(camera_bearing):.1f}deg")
            else: 
                print(f"Marker 7 not detected in camera view")

            next_print_time += 1.0

        if data.time >= next_ui_update:
            renderer.update_scene(data, camera="front_camera")
            disp_img = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
            cv2.imshow("TurtleBot3 Camera View", disp_img)
            cv2.waitKey(1)
            next_ui_update = data.time + 0.033

        viewer.sync()
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)       