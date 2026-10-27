import numpy as np

class TurtleBot3Controller:
    def __init__(self, track_width=0.288, wheel_radius=0.033):
        self.L = track_width
        self.r = wheel_radius
        
        self.k_rho = 0.8
        self.k_alpha = 2.0
        self.k_beta = -0.5
        
    def compute_control_command()