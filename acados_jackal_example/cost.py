import casadi as cd

def obj_control_input(u):
    # u: robot control input vector (normalized)
    cost = u.T @ u 

    return cost 



def obj_desired_pos(robot_pos, pos_start, pos_goal):
    dis_start_to_goal = (pos_goal - pos_start).T @ (pos_goal - pos_start)
    dis_normalize_factor = cd.fmax(dis_start_to_goal, 1)
    # dis_normalize_factor = 1
    dis_k_to_goal = (pos_goal - robot_pos).T @ (pos_goal - robot_pos)
    cost = dis_k_to_goal / dis_normalize_factor 

    return cost 


def obj_collision_potential(robot_pos, robot_size, obs_pos, obs_size):
    ell_axis = robot_size + 1.6*obs_size
    d_vec = obs_pos - robot_pos 
    d_vec_ell = d_vec / ell_axis        # elementwise division 
    d = d_vec_ell.T @ d_vec_ell
    cost = 1.0 / (1.0 + cd.exp(10.0*(d - 1.6)))

    return cost 



