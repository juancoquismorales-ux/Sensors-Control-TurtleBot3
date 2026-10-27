import numpy as np

# this si part A

class TurtleBot3Model:
    def __init__(self, track_width=0.288, wheel_radius=0.033):
        # define the physical parameters of the track from the tb file and xml
        self.L = track_width
        self.r = wheel_radius
        
    def kinematics(self, state, control, dt):
        """
        define the nonlinear unicycle model 
        with the state being x, y, theta. the position and heading angle
        and the control being the v, omega. the linear velocity and angular velocity
        """
        # set the states
        x, y, theta = state
        # set the controls
        v, omega = control
        
        # define the next values
        x_next = x + v * np.cos(theta) * dt
        y_next = y + v * np.sin(theta) * dt
        theta_next = theta + omega * dt
        
        # return it as an array
        return np.array([x_next, y_next, theta_next])
    
    def linearised_matrix(self, v_r, omega_r):
        """
        get the linearised matrix for state space A and B matrix for error dynbmia,cs linearise it
        and use it for the state errors e_x, e_y, and e_theta
        """
        A = np.array([
            [0.0, omega_r, 0.0],
            [-omega_r, 0.0, v_r],
            [0.0, 0.0, 0.0]
        ])
        
        B = np.array([
            [-1.0, 0.0],
            [0.0, 0.0],
            [0.0, -1.0]
        ])
        
        return A, B
    
    def check_controllability(self, A, B):
        """ 
        create the controlability matrix
        """
        
        n = A.shape[0]
        C = B
        A_power = np.eye(n)
        
        for i in range(1, n):
            A_power = A_power @ A
            C = np.hstack((C, A_power @ B))
            
        rank = np.linalg.matrix_rank(C)
        is_controllable = (rank == n)
        
        return is_controllable, rank, C
    
    def check_observability(self, A, C_obs):
        """ 
        create teh observability matrix O
        """
        
        n = A.shape[0]
        O = C_obs
        A_power = np.eye(n)
        
        for i in range(1, n):
            A_power = A_power @ A
            O = np.vstack((O, C_obs @ A_power))
            
        rank = np.linalg.matrix_rank(O)
        is_observable = (rank == n)
        
        return is_observable, rank, O
    
if __name__ == "__main__":
    robot_model = TurtleBot3Model()
    
    v_ref = 0.2
    omega_ref = 0.1
    
    A, B = robot_model.linearised_matrix(v_ref, omega_ref)
    
    controllable, rank_c, C_matrix = robot_model.check_controllability(A, B)
    print(f"part a model analysis")
    print(f"system controllabe: {controllable} rank: {rank_c}/3")
    
    H = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ])
    
    observable, rank_o, O_matrix = robot_model.check_observability(A, H)
    print(f"system observable: {observable} rank: {rank_o}/3")