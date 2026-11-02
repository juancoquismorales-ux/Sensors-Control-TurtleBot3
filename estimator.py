import numpy as np
# import our turtlebot model class
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
        # week 7 initial error covariance matrix P, which represents the uncertainty in our state estimate
        self.P = np.diag([0.1**2, 0.1**2, np.deg2rad(5)**2])
        # set how fast the uncertaintiy builds up over time during the motion of our turtlebot
        self.Q_rate = np.diag([0.01, 0.01, 0.005])
        # set teh measurement noise matrix (how much we trust teh sensors)
        self.R = np.diag([0.05, 0.05])
        # defines how much we trust camera vision measurements specificaly, for camera aruco range and bearing variance
        ## initial camera noise assumptions, These are starting valus used t validate, unmeasured calibration results
        self.R_marker = np.diag([0.05**2, np.deg2rad(2)**2]) # marker position and bearing noise
        
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
        # multiple the base uncertaintiy rate by timestep so the drift matches the loop duration
        ## scale the assumed process variance by the timestep to account for the discrete time update
        Q_step = self.Q_rate * self.dt 
        # grows the uncertainty matrix P forward using the motion jacobian
        self.P = F @ self.P @ F.T + Q_step 
        
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

# week 7 la h_rb H_rb and ekf_update
    # define a function to handle camera range and bearing math to correct our position estimate
    def update_with_marker(self, marker_position, measurement):
        # find the difference between marker and estimated state in the x 
        dx = marker_position[0] - self.x_hat[0]# calculate the difference in x between the marker and the estimated state
        # find the difference between marker and estimated state in the y
        dy = marker_position[1] - self.x_hat[1]# calculate the difference in y between the marker and the estimated state
        # square the differences to get the squared distance
        q = dx**2 + dy**2 # calculate the squared distance between the marker and the estimated state
        # square root the squared distance to get the predicted distance or the euclidean distance
        distance = np.sqrt(q) #predicted range to the marker

        # calculate the angle or bearing to marker relative to turtlebot heading
        bearing = np.arctan2(dy, dx) - self.x_hat[2] # calculate the bearing to the marker relative to the turtlebot's heading
        # noramlise the angles to -pi and pi
        bearing = np.arctan2(np.sin(bearing), np.cos(bearing)) # wrap the bearing to be within -pi and pi
        # store the predicted measurement vector for the euclidean distance and heading
        predicted_measurement = np.array([distance, bearing]) # create the predicted measurement vector

        # week 7 measurement jacobian
        H = np.array([[-dx / distance, -dy / distance, 0],
                      [dy / q, -dx / q, -1]]) 
# create the measurement jacobian matrix H based on the marker position and estimated state

        # calculate the residual error between actual measurement and predicted merasurement
        residual = measurement - predicted_measurement # calculate the residual error between actual and predicted measurement
        # normalise the bearing residual angle between -pi and pi
        residual[1] = np.arctan2(np.sin(residual[1]), np.cos(residual[1])) # wrap the bearing residual to be within -pi and pi

        # combine the uncertaintiy matrix P with camera noise matrix R_marker to calculate innovation covariance S
        S = H @ self.P @ H.T + self.R_marker # calculate the innovation covariance S
        # calculate the kalmain gain K using error covariance P, measurement jacobian H, and innovation covariance S to check how much we trust the measurement
        K = self.P @ H.T @ np.linalg.inv(S) # calculate the kalman gain K
        # applky the matrices to correct the state estimates using kalmain gain K and residual error
        self.x_hat = self.x_hat + K @ residual # update the state estimate using the kalman gain and residual
        # wrap the heading angle to be within -pi and pi
        self.x_hat[2] = np.arctan2(np.sin(self.x_hat[2]), np.cos(self.x_hat[2])) # wrap the heading angle to be within -pi and pi
        # shrink the uncertaintiy matrix P using kalmain gain K and measurement jacobian H to reduce uncertaintiy after updating
        self.P = (np.eye(3) - K @ H) @ self.P # update the error covariance matrix P to reduce uncertainty after marker update.

        # return teh updated state estiamte, and teh updated error covariance matrix P
        return self.x_hat, self.P # return the updated state estimate and covariance matrix

    
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