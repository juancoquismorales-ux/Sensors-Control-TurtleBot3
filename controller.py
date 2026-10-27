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
    
    # create function to convert the velocities to individual wheel actuator speeds
    def velocity_to_wheel(self, v, omega):
        # calculate teh left wheel angular velocity using differential drive equations
        omega_left = (v - (self.L / 2.0) * omega) / self.r
        # calculate the right wheel angular velocity using differential drive equations
        omega_right = (v + (self.L / 2.0) * omega) /self.r
        
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