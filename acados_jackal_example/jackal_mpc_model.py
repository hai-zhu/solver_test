import numpy as np
import casadi as cd 
from cost import obj_collision_potential, obj_desired_pos, obj_control_input

from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver 

def acados_jackal_mpc_model(pr, index):
    # Create an acados ocp model
    model = AcadosModel()

    # Name 
    model.name = 'jackal_mpc_ocp'

    # State
    x = cd.SX.sym('x', pr.nx)       # x = [px, py, theta]
    pos = x[index.x_pos]
    theta = x[index.x_theta]

    # State dot
    x_dot = cd.SX.sym('x_dot', pr.nx)   # x_dot = [px_dot, py_dot, theta_dot]

    # Control 
    u = cd.SX.sym('u', pr.nu)       # u = [vel, omega]
    vel = u[index.u_vel]
    omega = u[index.u_omega]

    # Params 
    param = cd.SX.sym('params', pr.nparam)
    p_robot_pos_start = np.array(pr.robot_pos_start)
    p_robot_pos_goal = np.array(pr.robot_pos_goal)
    p_robot_size = np.array(pr.robot_size)
    p_obs_pos = np.array(pr.obs_pos)
    p_obs_size = np.array(pr.obs_size)
    p_w_pos = pr.w_pos
    p_w_input = pr.w_input
    p_w_coll = pr.w_coll

    # p_robot_pos_start = param[index.p_robot_pos_start] 
    # p_robot_pos_goal = param[index.p_robot_pos_goal]
    # p_robot_size = param[index.p_robot_size]
    # p_obs_pos = param[index.p_obs_pos]
    # p_obs_size = param[index.p_obs_size]
    # p_mpc_weights = param[index.p_mpc_weights]
    # p_w_pos = p_mpc_weights[index.p_mpc_weights_w_pos]
    # p_w_input = p_mpc_weights[index.p_mpc_weights_w_input]
    # p_w_coll = p_mpc_weights[index.p_mpc_weights_w_coll]

    # Dynamics 
    dyn_f_expl = cd.vertcat(vel * cd.cos(theta), 
                            vel * cd.sin(theta), 
                            omega) 
    dyn_f_impl = x_dot - dyn_f_expl 

    # Cost terms 
    cost_wp_pos = obj_desired_pos(pos, p_robot_pos_start, p_robot_pos_goal)
    u_normalized = cd.vertcat(vel/pr.robot_maxVel, omega/pr.robot_maxOmega)
    cost_input = obj_control_input(u_normalized)
    cost_coll = obj_collision_potential(pos, p_robot_size, p_obs_pos, p_obs_size)
    # stage cost
    cost_stage = p_w_input*cost_input + p_w_pos*cost_wp_pos + p_w_coll*cost_coll
    # terminal cost 
    cost_e = p_w_pos * cost_wp_pos + p_w_coll*cost_coll

    # Path constraints
    a = p_robot_size[0] + 1.05*p_obs_size[0]
    b = p_robot_size[1] + 1.05*p_obs_size[1]
    pos_to_obs = p_obs_pos - pos  
    dis_pos_to_obs = pos_to_obs[0]**2 / a**2 + pos_to_obs[1]**2 / b**2 - 1.0

    # Formulating acados ocp model
    model.x = x 
    model.u = u 
    model.xdot = x_dot 
    model.p = param 
    model.f_expl_expr = dyn_f_expl 
    model.f_impl_expr = dyn_f_impl 
    model.cost_expr_ext_cost = cost_stage 
    model.cost_expr_ext_cost_e = cost_e 
    model.con_h_expr = dis_pos_to_obs
    # model.con_h_expr_e = dis_to_obs       # adding this constraint triggers error of QP

    return model 
