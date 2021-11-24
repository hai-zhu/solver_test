import numpy as np
import casadi as cd 

def obj_desired_pos(ego_pos, ego_start_pos, ego_goal_pos): 
    # Compute the goal position progressing cost, normalization is used
    lenToGoal   =   cd.dot(ego_goal_pos - ego_start_pos, ego_goal_pos - ego_start_pos)    
                                            # length between current start
                                            # and goal position, using
                                            #  quadratic form
    lenToGoal   =   cd.fmax(lenToGoal, 1)   # in case arriving at goal position
    Q = np.diag([1.0, 1.0]);                # x, y the same weight
    cost = (ego_goal_pos - ego_pos).T @ Q \
        @ (ego_goal_pos - ego_pos) / lenToGoal

    return cost 


def obj_input_acc(ego_input, pr):
    # Compute the control input cost, normalization is used
    ego_input_normalized = cd.vertcat(ego_input[0]/pr.robot_maxAx, ego_input[1]/pr.robot_maxAy)
    Q = np.diag([1.0, 1.0])	                # can be adjusted to penalize more along some direction
    cost = ego_input_normalized.T @ Q @ ego_input_normalized

    return cost 


def obj_collision_potential(ego_pos, ego_size, obs_pos, obs_size, obs_scale):
    # Compute the potential filed based collision avoidance cost
    a = ego_size[0] + obs_scale*obs_size[0]
    b = ego_size[1] + obs_scale*obs_size[1]
    
    d_vec = ego_pos - obs_pos
    d = d_vec[0]**2/a**2 + d_vec[1]**2/b**2
    # if else based
    # d_c = d - 1
    # cost = cd.if_else(d_c>0, 0, -d_c)
    # logistic based
    cost = 1 / (1+cd.exp(10*(d - obs_scale)))

    return cost 


