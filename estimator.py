import numpy as np
# import our robot model class
from model import TurtleBot3Model

# create class for ekf for the turtlebot 
class TurtleBot3EKF:
    # define the constructor or insant parameters, matrices adn timesteps
    def __init__(self, dt=0.01):
        # set teh timestep dt
        self.dt = dt
        # set teh model to the turtlebot model
        self.model = TurtleBot3Model()
        # initialise the estimated state vector to zero for x, y, theta
        self.x_hat = np.zeros(3)
        # initialise the error covariance matrix P with an initial uncertainty
        self.P = np.diag([0.1, 0.1, 0.1])
        # set the process nosie matrix Q (how much we trust the motion model)
        self.Q = np.diag([0.01, 0.01, 0.005])
        # set teh measurement noise matrix (how much we trust teh sensors)
        self.R = np.diag([0.05, 0.05])
        
    # create function to predict the next state using teh control v and omega
    def predict(self, v, omega):
        # unpack the current estimated state values
        x, y, theta = self.x_hat
        
        # predict the next state using teh model kinematics and control inputs from our model class
        self.x_hat = self.model.kinematics(self.x_hat, np.array([v, omega]), self.dt)
        
        # create F matrix or jacobian with respect to state
        F = np.array([
            [1.0, 0.0, -v * np.sin(theta) * self.dt],
            [0.0, 1.0, v * np.cos(theta) * self.dt],
            [0.0, 0.0, 1.0]
        ])
        
        # update the error covariance matrix using the prediction equation --> F * P @ F^T + Q
        self.P = F @ self.P @ F.T + self.Q
        
        # return the predicted state vector
        return self.x_hat
    
    # create corrected function to combine sensor measurements with our prediction
    def update(self, z_measurement, H):
        # predict teh sensor reading using the measurement matrix H and estimated state
        z_pred = H @ self.x_hat
        # calculate teh residual error between actual sensor reading and prediction
        y = z_measurement - z_pred
        # calculate the innovation covariance S
        S = H @ self.P @ H.T + self.R
        # calculat ekalmain gain factor
        K = self.P @ H.T @ np.linalg.inv(S)
        # update the state estimate by adding kalmain gain multiplied by the residual error
        self.x_hat = self.x_hat + K @ y
        # normalise the heading angle theta to stay within -pi and pi
        self.x_hat[2] = np.arctan2(np.sin(self.x_hat[2]), np.cos(self.x_hat[2]))
        # create identity matrix for the state size
        I = np.eye(len(self.x_hat))
        # update the error covariance matrix P to reduce uncertaintiy after sensor update
        self.P = (I - K @ H) @ self.P
        
        # return updated state estimate and new covariance matrix P
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