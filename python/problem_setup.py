## probelm setup for the collision avoidance scenario
import numpy as np
from dataclasses import dataclass

@dataclass
class Problem:
    # workspace environment
    ws_x = [-10.0, 10.0]
    ws_y = [-6.0, 6.0]
    # robot physics
    robot_maxVx = 2.0
    robot_maxVy = 2.0
    robot_maxAx = 1.0
    robot_maxAy = 1.0
    # obstacle physics
    obs_size = [1.6, 1.0]
    obs_scale = 1.6
    # robot start and goal positions
    robot_size = [0.4, 0.4]
    robot_pos_start = [-8.0, 0.0]
    robot_pos_goal = [8.0, 0.0]
    # obstacle pos
    obs_pos = [0, -0.1]
    # MPC parameters
    dt              = 0.1
    N               = 20
    w_pos           = 8.0               # MPC weights
    w_inputs        = 0.0
    w_coll          = 0.0
    w_slack         = 1E4


@dataclass
class Index:
    # z vector
    z_inputs    = []
    z_slack     = []
    z_pos       = []
    z_vel       = []
    # x vector
    x_pos       = []
    x_vel       = []
    # p vector
    p_robot_state   = []
    p_robot_start   = []
    p_robot_goal    = []
    p_robot_size    = []
    p_mpc_weights   = []
    p_obs_pos       = []
    p_obs_size      = []
    p_obs_scale     = []


@dataclass
class Mpc_model:
    # MPC
    dt = 0             # sampling time
    N  = 0             # horizon length
    nx = 0             # number of equality constraints, x
    nu = 0             # number of control inputs, u
    ns = 0             # number of slacks, s
    np = 0             # number of real-time params on each stage, 
    # bound 
    xl = []
    xu = []
    ul = []
    uu = []
    sl = []
    su = []
