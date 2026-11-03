import numpy as np

# create class for waypoint tracking controller
class TurtleBot3Controller:
    # define the instant with track width, wheel radius and tning gains
    def __init__(self, track_width=0.288, wheel_radius=0.033):
        # store the track width
        self.L = track_width
        # store the wheel radius
        self.r = wheel_radius
        
        # boolean to check if the obstacle avoidance is active or not
        self.avoiding = False
        # store the direction of avoidance, 1 for left, -1 for right. set initially to the left
        self.avoid_direction = 1.0
        # store the goal distance when avoidance starts
        self.avoid_start_distance = np.inf
        # store the heading angle for the initila turn for avoidance
        self.avoid_heading = None
        # emergency turn 
        self.blocked_turn = None

        # set the gain for distance error (controls v)        
        self.k_rho = 0.8
        # set teh gain for heading angle error (controls omega)
        self.k_alpha = 2.0
        
    # create function to calculate linear and angular velocity commands to reach desired waypoint
    def compute_control_command(self, current_state, target_waypoint):
        # unpack current estimated pose x, y, and theta from ekf state
        x, y, theta = current_state
        # unpack the target pose
        x_goal, y_goal, theta_goal = target_waypoint
        
        # calculate the x distance difference to waypoint
        dx = x_goal - x
        # calculate the y distance difference to waypoint
        dy = y_goal - y
        
        # calculate straight line distance error or rho to waypoint using pythag
        rho = np.sqrt(dx**2 + dy**2)
        
        # if the straight line distance error or rho is close enough (less than 5 cm), stop if reached
        if rho < 0.05: 
            return 0.0, 0.0
        
        # calculate the angle to target, subtracting current heading to get alpha error
        alpha = np.arctan2(dy, dx) - theta
        # normalise the alpha angle to stay between -pi and pi
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        omega = self.k_alpha * alpha
        omega = np.clip(omega, -2.84, 2.84)
        
        if abs(alpha) > np.deg2rad(30):
            v = 0.0
        
        else:
            v = self.k_rho * rho * np.cos(alpha)
            # set a limi to the lniear speed v to the limits of the waffle pi
            v = np.clip(v, 0.0, 0.22)

        # return teh calculated linear velocity and angular velocity commands
        return v, omega
    
    # create function to check lidar readings and also handle obstacle avoidance commands
    def compute_obstacle_avoidance_command(self, current_state, target_waypoint, lidar_ranges, threshold=0.4):
        # if the lidar ranges are none or not equal to 360 degrees do this
        if lidar_ranges is None or len(lidar_ranges) != 360:
            # return the linear and angular velocity as 0
            return 0.0, 0.0
        
        # unpack the estimated pose x, y, theta from the current state
        x, y, theta = current_state
        # calculate the x difference to waypoint
        dx = target_waypoint[0] - x
        # calculate the y difference to waypoint
        dy = target_waypoint[1] - y
        # calculate the straight line goal distance using hypotenuse or euclidean distance
        goal_distance = np.hypot(dx, dy)
        # calculate the goal angle by minus the heading
        goal_angle = np.arctan2(dy, dx) - theta
        
        # combine the rear right and front left segments to form a front avoidance cone
        avoidance_cone = np.concatenate((lidar_ranges[300:], lidar_ranges[:60]))
        # get the minimum distance in front of the turtle bot within the avoidance cone
        front_dist = np.min(avoidance_cone)
        
        # if we ar enot avoiding an obstacle do this
        if not self.avoiding:
            # if the front distance is greater or equal to the threshold do this
            if front_dist >= threshold:
                # return the normal control commands to waypoint if the path is clear
                return self.compute_control_command(current_state, target_waypoint)

                left_clearance = np.mean(np.minimum(lidar_ranges[60:120], 6))
                right_clearance = np.mean(np.minimum(lidar_ranges[240:300], 6))

                # determine the avoidance direction, 1.0 to the left and -1.0 to the right
                self.avoid_direction = (1.0 if left_clearance >= right_clearance else -1.0)
            
            # set the boolean flag aoviding to true to start avoidance
            self.avoiding = True
            # set the goal distance when obstacle avoidance is initiated
            self.avoid_start_distance = goal_distance
            
            # calculat eth eheading angle for initial avoidance turn
            self.avoid_heading = theta + self.avoid_direction * (np.pi / 2)
            
        # if we are avoiding, do this
        if self.avoid_heading is not None:
            # calculate the turn error between teh avoidance heading and current heading theta
            turn_error = self.avoid_heading - theta
            # normalise the turn error angle between -pi and pi
            turn_error = np.arctan2(np.sin(turn_error), np.cos(turn_error))
            
            # if the absolute turn error is greater than 0.1 radians do this
            if abs(turn_error) > 0.1:
                # return teh linear velocity as 0, and clip the angular velocity for turning
                return 0.0, np.clip(1.5 * turn_error, -0.8, 0.8)
            
            # reset teh avoiding state to none once turning is complete
            self.avoid_heading = None

        # create an array of angles in radians for all 360 degrees
        angles = np.deg2rad(np.arange(360))
        # create a boolean mask to check for finite and valid lidar ranges
        valid = np.isfinite(lidar_ranges)
        # calculate the x coordinate of lidar scan points in teh turtlebots frame, accounting for sensor offset
        scan_x = -0.049 + lidar_ranges[valid] * np.cos(angles[valid])
        # calculate the y coordinates of lidar scan points in the turtlebots frame
        scan_y = lidar_ranges[valid] * np.sin(angles[valid])
        
        # calculate the left and right coordinate along the goal vector direction
        along = scan_x * np.cos(goal_angle) + scan_y * np.sin(goal_angle)
        # calculate the up and down coordinate across the goal vector direction
        across = -scan_x * np.sin(goal_angle) + scan_y * np.cos(goal_angle)
        
        # check if any obstacle points block the direct route to the goal
        route_blocked = np.any((along > 0.0) & (along < goal_distance) & (np.abs(across) < 0.30))

        if not route_blocked and front_dist >= threshold:
            self.avoiding = False
            self.avoid_heading = None
            return self.compute_control_command(current_state, target_waypoint)
        # if the front distance is less than the threshold, do this
        if front_dist < threshold:
            # match the order of rays in teh avoidance cone to -60 to 60 degrees
            front_angles = np.arange(-60, 60)
            # find which side where the obstacle is
            obstacle_angle = front_angles[np.argmin(avoidance_cone)]

            # if teh obstacle angle is greater than 0 do this
            if obstacle_angle > 0:
                # turn to the right
                self.avoid_direction = -1.0
            # else if its less than 0
            elif obstacle_angle < 0:
                # turn to the left
                self.avoid_direction = 1.0

            # return linear velocity as 0, and scale the angular velocity avoidance direction
            return 0.0, self.avoid_direction * 0.8
        
        # if the avoid direction from lidar is greater than 0 do this
        if self.avoid_direction > 0:
            # select the right side indices if the wall is on the right so turn to the left
            side_indices = np.arange(240, 330)
        # else do this
        else:
            # select teh left side indices if the wall is on the left so turn to the right
            side_indices = np.arange(30, 120)
            
        # get teh side lidar ranges capped at 6 metres
        side_ranges = np.minimum(lidar_ranges[side_indices], 6.0)
        # convert the side indices angles to radians
        side_angles = np.deg2rad(side_indices)
        # calculate the minimum side distance to maintain safe distance from obstacle wall
        side_distance = np.min(side_ranges * np.abs(np.sin(side_angles)))
            
        # calculate the angular velocity omega proportional to the clearnace error on teh side
        omega = self.avoid_direction * 2.0 * (0.35 - side_distance)
        # clip the angular velocity omega within the safe limits of the turtlebot
        omega = np.clip(omega, -0.8, 0.8)
        
        # return a slow forward linear velocity and teh calculated angular velocity omega command
        return 0.10, omega
    
    # create function to convert the velocities to individual wheel actuator speeds
    def velocity_to_wheel(self, v, omega):
        # calculate teh left wheel angular velocity using differential drive equations
        omega_left = (v - (self.L / 2.0) * omega) / self.r
        # calculate the right wheel angular velocity using differential drive equations
        omega_right = (v + (self.L / 2.0) * omega) /self.r

        # scale down both wheel speeds proportionally if eithe rexceeds the maximum actuator speed of 7.88 rad/s, so turtlebot stays on arc
        ## scale the wheel speeds if they exceed the maximum allowed speed of 7.88 rad/s
        scale = max(1, abs(omega_left) / 7.88, abs(omega_right) / 7.88)
        # apply the wheel speed scaling to the left wheel command
        omega_left /= scale
        # apply the wheel speed scaling to the right wheel command
        omega_right /= scale
        
        # return teh left and right wheel speeds for the actuators
        return omega_left, omega_right
    
if __name__ == "__main__":
    controller = TurtleBot3Controller()
    
    estimated_state = np.array([0.0, 0.0, 0.0])
    
    waypoint = np.array([1.0, 1.0, 0.0])
    
    v, omega = controller.compute_control_command(estimated_state, waypoint)
    left_speed, right_speed = controller.velocity_to_wheel(v, omega)
    
    print(f"controller test")
    print(f"velocity commands (v, omega): v = {v:.3f} m/s, omega = {omega:.3f} rad/s")
    print(f"actuator wheel speeds: left = {left_speed:.3f} rad/s, right = {right_speed:.3f} rad/s")