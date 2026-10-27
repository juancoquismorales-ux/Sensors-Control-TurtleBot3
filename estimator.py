import numpy as np
from model import TurtleBot3Model

class TurtleBot3EKF:
    def __init__(self, dt=0.01):
        self.dt = dt
        
        self.model = TurtleBot3Model()
        
        self.x_hat = np.zeros(3)
        
        self.P = np.diag([0.1, 0.1, 0.1])
        
        self.Q = np.diag([0.01, 0.01, 0.005])
        
        self.R = np.diag([0.05, 0.05])
        
    def predict(self, v, omega):
        x, y, theta = self.x_hat
        
        self.x_hat = self.model.kinematics(self.x_hat, np.array([v, omega]), self.dt)
        
        F = np.array([
            [1.0, 0.0, -v * np.sin(theta) * self.dt],
            [0.0, 1.0, v * np.cos(theta) * self.dt],
            [0.0, 0.0, 1.0]
        ])
        
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x_hat
    
    def update(self, z_measurement, H):
        z_pred = H @ self.x_hat
        y = z_measurement - z_pred
        
        S = H @ self.P @ H.T + self.R
        
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x_hat = self.x_hat + K @ y
        
        self.x_hat[2] = np.arctan2(np.sin(self.x_hat[2]), np.cos(self.x_hat[2]))
        
        I = np.eye(len(self.x_hat))
        self.P = (I - K @ H) @ self.P
        
        return self.x_hat, self.P
    
if __name__ == "__main__":
    ekf = TurtleBot3EKF(dt=0.01)
    
    predicted_state = ekf.predict(v=0.2, omega=0.0)
    print(f"ekf test")
    print(f"predicted state (x, y, theta): {predicted_state}")
    
    H_matrix = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ])
    sensor_reading = np.array([0.002, 0.0])
    updated_state, updated_covariance = ekf.update(sensor_reading, H_matrix)
    print(f"updated state after sensor fusion: {updated_state}")