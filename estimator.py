import numpy as np
from model import TurtleBot3Model

class TurtleBot3EKF:
    def __init__(self, dt=0.01):
        self.model = TurtleBot3Model()
        
        self.x_hat = np.zeros(3)
        
        self.P = np.diag([0.1, 0.1, 0.1])
        
        self.Q = np.diag([0.01, 0.01, 0.005])
        
        self.R = np.diag([0.05, 0.05])