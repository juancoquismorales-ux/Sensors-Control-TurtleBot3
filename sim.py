import time
import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
from model import TurtleBot3Model
from estimator import TurtleBot3EKF
from controller import TurtleBot3Controller

model_path = (Path(__file__).parent / "robotis_tb3" / "scene_turtlebot3_waffle_pi.xml")

model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

dt, track_width, wheel_radius = 0.002, 0.288, 0.033
ekf = TurtleBot3EKF(dt=dt)
controller = TurtleBot3Controller(track_width=track_width, wheel_radius=wheel_radius)

waypoints = [
    np.array([1.0, 0.0, 0.0]),
    np.array([1.0, 1.0, 0.0]),
    np.array([0.0, 1.0, 0.0]),
    np.array([0.0, 0.0, 0.0])
]
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

v_cmd, omega_cmd = 0.0, 0.0
next_print_time = 1.0

print(f"starting mujoco ekf and controller sim")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        
        estimated_state = ekf.predict(v_cmd, omega_cmd)
        
        left_angle = float(data.joint("wheel_left").qpos[0])
        right_angle = float(data.joint("wheel_right").qpos[0])
        
        dphi_L = left_angle - previous_left
        dphi_R = right_angle - previous_right
        
        previous_left = left_angle
        previous_right = right_angle
        
        d_forward = 0.5 * wheel_radius * (dphi_L + dphi_R)
        
        sensor_measurement = np.array([
            estimated_state[0] + d_forward * np.cos(estimated_state[2]),
            estimated_state[1] + d_forward * np.sin(estimated_state[2])
        ])
        
        estimated_state, _ = ekf.update(sensor_measurement, H_matrix)
        
        distance_to_goal = np.linalg.norm(target_waypoint[:2] - estimated_state[:2])
        if distance_to_goal < 0.1 and waypoint_idx < len(waypoints) - 1:
            waypoint_idx += 1
            target_waypoint = waypoints[waypoint_idx]
            print(f"reached waypoint. switching to target index {waypoint_idx}: {target_waypoint}")
            
        v_cmd, omega_cmd = controller.compute_control_command(estimated_state, target_waypoint)
        
        left_speed, right_speed = controller.velocity_to_wheel(v_cmd, omega_cmd)
        
        data.ctrl[left_actuator_id] = left_speed
        data.ctrl[right_actuator_id] = right_speed
        
        mujoco.mj_step(model, data)
        
        if data.time >= next_print_time:
            base_pose = data.joint("base_joint").qpos.copy()
            true_x, true_y = float(base_pose[0]), float(base_pose[1])
            
            print(f"t = {data.time:1f}s | est pos: ({estimated_state[0]:.3f}, {estimated_state[1]:.3f}) | target index: {waypoint_idx}")
            next_print_time += 1.0
            
        viewer.sync()
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)       