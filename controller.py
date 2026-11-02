import numpy as np

# create class for waypoint tracking controller
class TurtleBot3Controller:
    # define the instant with track width, wheel radius and tning gains
    def __init__(self, track_width=0.288, wheel_radius=0.033):
        # store the track width
        self.L = track_width
        # store the wheel radius
        self.r = wheel_radius

        # set the gain for distance error (controls v)        
        self.k_rho = 0.8
        # set teh gain for heading angle error (controls omega)
        self.k_alpha = 2.0
        # set gain for final target orientation alignment
        self.k_beta = -0.5
        
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
        
        # calculate the final orientation heading error beta
        beta = theta_goal - theta
        # normalise the beta angle to stay between -pi and pi
        beta = np.arctan2(np.sin(beta), np.cos(beta))
        
        # calculate the linear velocity v proportional to teh distance error rho
        v = self.k_rho * rho
        # calculate the angular velocity omega using heading errors alpha an dbeta
        omega = self.k_alpha * alpha + self.k_beta * beta
        
        # set a limi to the lniear speed v to the limits of the waffle pi
        v = np.clip(v, -0.22, 0.22)
        # set a limit to teh angular velocity omega to the limits of the waffle pi
        omega = np.clip(omega, -2.84, 2.84)
        
        # return teh calculated linear velocity and angular velocity commands
        return v, omega
    
    # create function to check lidar readings and adjust linear and angular velocity commands to avoid obstacles
    def compute_obstacle_avoidance_command(self, current_state, target_waypoint, lidar_ranges, threshold=0.4):
        # check if the ldiar ranges are available and in 360 degrees
        if lidar_ranges is not None and len(lidar_ranges) == 360:
            # check the avoidance cone or front cone sector from 330 to 360 degrees, as well as 0 to 30 degrees for obstaces
            avoidance_cone = np.concatenate((lidar_ranges[330:], lidar_ranges[:30]))
            # find the minimum distance in front of the turtlebot
            min_dist = np.min(avoidance_cone)
            
            # if there is an obstacle closer than the minimum or safety distance, do this
            if min_dist < threshold:
                # set the lineear velocity to 0 and stop moving
                v = 0.05
                
                # calculate average clearnace on left side of turtlebot using lidar ranges from 60 to 120 degrees 
                left_clearance = np.mean(lidar_ranges[60:120])
                # calculate average clearnaces on right side of turtlebot using lidar ranges from 240 to 330 degrees
                right_clearance = np.mean(lidar_ranges[240:330])
                # if the left side has more clearance than teh right side, do this
                if left_clearance >= right_clearance:
                    # set the angular velocity to turn left to avoid obstacle at 1.5 rad/s
                    omega = 1.5
                # else or if the right side has more clearnace than the left side, do this
                else:
                    # set the angular velocity to turn right to avoid obstacle at -1.5 rad/s
                    omega = -1.5
                
                # return the linera and angular velocity commands to avoid the obstacle
                return v, omega
            
        # if no obstacles are detected, compute the normal control commands to waypoint
        return self.compute_control_command(current_state, target_waypoint)
    
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