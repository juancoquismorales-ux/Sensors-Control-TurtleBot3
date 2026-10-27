import numpy as np

# this si part A

# define class for turtlebot 3 kinematic model
class TurtleBot3Model:
    # define the instant of the class constructor, with defined track width and wheel radius
    def __init__(self, track_width=0.288, wheel_radius=0.033):
        # store the track width 
        self.L = track_width
        # store the wheel radius
        self.r = wheel_radius
        
    # create function to compute the nonlinear unicycle states over time
    def kinematics(self, state, control, dt):
        # unpack the current x, y, and theta from the state
        x, y, theta = state
        # unpack teh linear speed v and angular velocity omega from the control
        v, omega = control

        # calculate the next x position        
        x_next = x + v * np.cos(theta) * dt
        # calculate the next y position
        y_next = y + v * np.sin(theta) * dt
        # calculate the next heading angle theta
        theta_next = theta + omega * dt

        # return the next state as an array        
        return np.array([x_next, y_next, theta_next])
    
    # create function to linearise the state-space matrix A and B for error math
    def linearised_matrix(self, state_r, control_r):
        _, _, theta_r = state_r
        v_r, _ = control_r
        # define A matrix using reference velocities
        A = np.array([
            [0.0, 0.0, -v_r * np.sin(theta_r)],
            [0.0, 0.0, v_r * np.cos(theta_r)],
            [0.0, 0.0, 0.0]
        ])
    
        # define B matrix for control        
        B = np.array([
            [np.cos(theta_r), 0.0],
            [np.sin(theta_r), 0.0],
            [0.0, -1.0]
        ])
        
        # return both A and B matrices
        return A, B
    
    # create function to check system contrallability using the kalman rank
    def check_controllability(self, A, B):
        # get teh number of states from matrix A size
        n = A.shape[0]
        # set teh initial controllability matrix C to matrix B
        C = B
        # create identity matrix for the powers of matrix A
        A_power = np.eye(n)
        
        # for loop to create the full controllability matrix
        for i in range(1, n):
            # multiply current power by A
            A_power = A_power @ A
            # add teh new column to the controllability matrix C
            C = np.hstack((C, A_power @ B))
        
        # find the rank of matrix C            
        rank = np.linalg.matrix_rank(C)
        # check if rank equals the number of states, if so is controllable
        is_controllable = (rank == n)
        
        # return the check, rank and matrix C
        return is_controllable, rank, C
    
    # create function to check if the system can be observed by sensors
    def check_observability(self, A, C_obs):
        # get number of states from matrix A size
        n = A.shape[0]
        # set initial observability matrix to sensor matrix
        O = C_obs
        # create identity matrix for powers of A
        A_power = np.eye(n)
        
        # for loop to create the full observability matrix
        for i in range(1, n):
            # multiply current power by A
            A_power = A_power @ A
            # add teh new row block to observability matrix O
            O = np.vstack((O, C_obs @ A_power))
            
        # find the rank of matrix O
        rank = np.linalg.matrix_rank(O)
        # check if the rank equals the number of states, if so is observable by sensor
        is_observable = (rank == n)
        
        # return the check, rank and matrix O
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