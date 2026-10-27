import numpy as np

class TurtleBot3Controller:
    def __init__(self, track_width=0.288, wheel_radius=0.033):
        self.L = track_width
        self.r = wheel_radius
        
        self.k_rho = 0.8
        self.k_alpha = 2.0
        self.k_beta = -0.5
        
    def compute_control_command(self, current_state, target_waypoint):
        x, y, theta = current_state
        x_goal, y_goal, theta_goal = target_waypoint
        
        dx = x_goal - x
        dy = y_goal - y
        
        rho = np.sqrt(dx**2 + dy**2)
        
        if rho < 0.05: 
            return 0.0, 0.0
        
        alpha = np.arctan2(dy, dx) - theta
        alpha = np.arctan2(np.sin(alpha), np.cos(alpha))
        
        beta = theta_goal - theta
        beta = np.arctan2(np.sin(beta), np.cos(beta))
        
        v = self.k_rho * rho
        omega = self.k_alpha * alpha + self.k_beta * beta
        
        v = np.clip(v, -0.22, 0.22)
        omega = np.clip(omega, -2.84, 2.84)
        
        return v, omega
    
    def velocity_to_wheel(self, v, omega):
        omega_left = (v - (self.L / 2.0) * omega) / self.r
        omega_right = (v + (self.L / 2.0) * omega) /self.r
        
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