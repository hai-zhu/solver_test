import numpy as np 
from dataclasses import dataclass 


# Problem setup 
@dataclass
class Problem:
    # workspace 
    ws_x = [-10.0, 10.0]    # m
    ws_y = [-6.0, 6.0]      # m
    # robot control bound
    robot_maxVel = 1.0      # m/s 
    robot_maxOmega = 1.0    # rad/s
    robot_maxAcc = 3.0      # m / s ^ 2
    robot_maxOmegaAcc = 1.0  # rad / s ^ 2
    # robot size, start and goal positions, can be real time param
    robot_size = [0.5, 0.3] # ellipse, m
    robot_pos_start = [-8.0, 0.0]   # m
    robot_theta_start = [np.deg2rad(60.0)]  # rad
    robot_pos_goal = [8.0, 0.0]     # m
    # obstacle size, position, can be real time param 
    obs_size = [1.6, 1.0]   # m
    obs_pos = [0.0, -0.1]   # m
    # MPC settings
    dt = 0.1                # sampling time, s
    N = 20                  # horizon length
    T = N*dt                # horizon time
    nx = 5                  # state dimension
    nu = 2                  # control dimension
    ns = 0                  # slack dimension
    nvar = nu + ns + nx     # nb. of variables in z vector
    neq = nx                # nb. of equality constraints
    nh = 1                  # nb. of inequality constraints
    nparam = 14             # parameter dimension
    # MPC cost terms weights, can be real time param
    w_pos = 8.0
    w_yaw = 0.01
    w_input = 0.01
    w_coll = 0.1
    

# Vector index 
@dataclass
class Index:
    # state vector, x = [px, py, theta, vel, omega]
    x_pos = np.s_[0: 2]     # 0, 1
    x_theta = np.s_[2]      # 2
    x_vel = np.s_[3]        # 3
    x_omega = np.s_[4]      # 4
    # control vector, u = [acc, omega_acc]
    u_acc = np.s_[0]        # 0
    u_omega_acc = np.s_[1]  # 1
    # z vector
    z_inputs = np.s_[0: 2]  # 0, 1
    z_states = np.s_[2: 7]  # 2, 3, 4, 5, 6
    # param vector
    p_robot_pos_start = np.s_[0: 2]     # 0, 1
    p_robot_pos_goal = np.s_[2: 4]      # 2, 3
    p_robot_size = np.s_[4: 6]          # 4, 5
    p_obs_pos = np.s_[6: 8]             # 6, 7
    p_obs_size = np.s_[8: 10]           # 8, 9
    p_mpc_weights = np.s_[10: 14]       # 10, 11, 12, 13
    p_mpc_weights_w_pos = np.s_[0]      # 0 in mpc_weights
    p_mpc_weights_w_yaw = np.s_[1]      # 1 in mpc_weights
    p_mpc_weights_w_input = np.s_[2]    # 2 in mpc_weights
    p_mpc_weights_w_coll = np.s_[3]     # 3 in mpc_weights
