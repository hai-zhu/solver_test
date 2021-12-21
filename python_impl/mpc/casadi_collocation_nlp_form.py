import casadi as cd

import sys
import os
from pathlib import Path

FILE_THIS = Path(__file__).resolve()
PARENT = FILE_THIS.parent
GPARENT = FILE_THIS.parents[1]
sys.path.append(str(PARENT))
sys.path.append(str(GPARENT))

from objectives import obj_mpc

def casadi_collocation_nlp_form(model, index, pr):
    
    ## Define stage variables
    # state
    px = cd.MX.sym('px')
    py = cd.MX.sym('py')
    vx = cd.MX.sym('vx')
    vy = cd.MX.sym('vy')
    x_stage = cd.vertcat(px, py, vx, vy)
    # control
    ax = cd.MX.sym('ax')
    ay = cd.MX.sym('ay')
    u_stage = cd.vertcat(ax, ay)
    # slack
    s_ca = cd.MX.sym('s_ca')
    s_stage = s_ca
    # parameters
    p_stage = cd.MX.sym('p', model.np)
    p_rob_start = p_stage[index.p_robot_start]
    p_rob_goal  = p_stage[index.p_robot_goal]
    p_rob_size  = p_stage[index.p_robot_size]
    p_mpc_weights = p_stage[index.p_mpc_weights]
    p_obs_pos   = p_stage[index.p_obs_pos]
    p_obs_size  = p_stage[index.p_obs_size]
    p_obs_scale = p_stage[index.p_obs_scale]
    # mpc weights
    w_pos       =  p_mpc_weights[0]
    w_inputs    =  p_mpc_weights[1]
    w_coll      =  p_mpc_weights[2]
    w_slack     =  p_mpc_weights[3]


    ## Define model equations
    px_dot = vx 
    py_dot = vy 
    vx_dot = ax 
    vy_dot = ay 
    x_stage_dot = cd.vertcat(px_dot, py_dot, vx_dot, vy_dot) 
    dyn_f = cd.Function('dyn_f', [x_stage, u_stage], [x_stage_dot])


    ## Define objective function
    # control input cost
    cost_input_acc = w_inputs * obj_mpc.obj_input_acc(u_stage, pr)
    obj_input = cd.Function('obj_input', [u_stage, p_stage], [cost_input_acc])
    # waypoint navigation cost
    cost_wp_pos = w_pos * obj_mpc.obj_desired_pos(x_stage[index.x_pos], p_rob_start, p_rob_goal)
    obj_wp = cd.Function('obj_wp', [x_stage, p_stage], [cost_wp_pos])
    # collision potentials cost
    cost_coll = w_coll * obj_mpc.obj_collision_potential(x_stage[index.x_pos], p_rob_size, p_obs_pos, p_obs_size, p_obs_scale)
    obj_coll = cd.Function('obj_coll', [x_stage, p_stage], [cost_coll])
    # slack cost
    cost_slack = w_slack * s_stage**2
    obj_slack = cd.Function('obj_slack', [s_stage, p_stage], [cost_slack])


    ## Define inequality constraints function
    a = p_rob_size[0] + 1.05*p_obs_size[0]
    b = p_rob_size[1] + 1.05*p_obs_size[1]
    d = x_stage[index.x_pos] - p_obs_pos
    ego_obs_dis = d[0]**2/a**2 + d[1]**2/b**2 -1 + s_stage    # ego_obs_dis > 0
    h_ego_obs_dis = cd.Function('h_ego_obs_dis', [x_stage, p_stage, s_stage], [ego_obs_dis]) 


    # Definitions for collocation
    DT = model.dt
    XK = cd.MX.sym('XK', model.nx)
    XK1 = cd.MX.sym('XK1', model.nx)
    XDOTK = cd.MX.sym('XDOTK', model.nx)
    XDOTK1 = cd.MX.sym('XDOTK1', model.nx)
    UK = cd.MX.sym('UK', model.nu)
    UK1 = cd.MX.sym('UK1', model.nu)
    X_tc = 1./2 * (XK + XK1) + DT/8 * (XDOTK - XDOTK1)
    f_x_tc = cd.Function('f_x_tc', [XK, XK1, XDOTK, XDOTK1], [X_tc], ['xk', 'xk1', 'xdk', 'xdk1'], ['xtc'])
    Xdot_tc = -3/(2*DT)*(XK - XK1) - 1./4 * (XDOTK + XDOTK1)
    f_xdot_tc = cd.Function('f_xdot_tc', [XK, XK1, XDOTK, XDOTK1], [Xdot_tc], ['xk', 'xk1', 'xdk', 'xdk1'], ['xdtc'])
    U_tc = 1/2 * (UK + UK1)
    f_u_tc = cd.Function('f_u_tc', [UK, UK1], [U_tc], ['uk', 'uk1'], ['utc'])


    # NLP Forumlation
    nlp_x   = []            # decision variables
    nlp_x0  = []            # initial guess
    nlp_lbx = []            # lower bound on decision variables
    nlp_ubx = []            # upper bound on decision variables
    nlp_J   = 0             # cost accumulator
    nlp_g   = []            # inequality constraint equations
    nlp_lbg = []            # lower bound for g
    nlp_ubg = []            # upper bound for g
    nlp_p   = []            # real time parameters

    # Lift initial conditions
    Xk = cd.MX.sym('X0', model.nx)
    nlp_x   += [Xk]
    nlp_lbx += [0] * model.nx 
    nlp_ubx += [0] * model.nx 
    nlp_x0  += [0] * model.nx 

    Uk = cd.MX.sym('U0', model.nu)
    nlp_x   += [Uk]
    nlp_lbx += model.ul
    nlp_ubx += model.uu 
    nlp_x0  += [0] * model.nu 

    # Collocation 
    for k in range(model.N):

        # Parameters
        Pk = cd.MX.sym('P_' + str(k), model.np)     # parameter of this stage
        nlp_p += [Pk]

        # New NLP variables for the state
        Xk_next = cd.MX.sym('X_' + str(k+1), model.nx)
        nlp_x += [Xk_next]
        nlp_lbx += model.xl
        nlp_ubx += model.xu 
        nlp_x0  += [0] * model.nx 

        # New NLP variables for the control
        Uk_next = cd.MX.sym('U_' + str(k+1), model.nu)
        nlp_x += [Uk_next]
        nlp_lbx += model.ul 
        nlp_ubx += model.uu 
        nlp_x0 += [0] * model.nu 

        # Collocation constraints
        Xdot_k = dyn_f(Xk, Uk)
        Xdot_k1 = dyn_f(Xk_next, Uk_next)
        x_col = f_x_tc(xk=Xk, xk1=Xk_next, xdk=Xdot_k, xdk1=Xdot_k1)['xtc']
        u_col = f_u_tc(uk=Uk, uk1=Uk_next)['utc']
        xdot_col = f_xdot_tc(xk=Xk, xk1=Xk_next, xdk=Xdot_k, xdk1=Xdot_k1)['xdtc']
        nlp_g += [xdot_col - dyn_f(x_col, u_col)]
        nlp_lbg += [0] * model.nx 
        nlp_ubg += [0] * model.nx 

        # Slack 
        Sk = cd.MX.sym('S_'+ str(k+1), model.ns)
        nlp_x += [Sk]               # adding sk as decision variable
        nlp_lbx += model.sl
        nlp_ubx += model.su  
        nlp_x0 += [0] * model.ns 

        # Add path constraint
        nlp_g += [h_ego_obs_dis(Xk, Pk, Sk)]
        nlp_lbg += [0]
        nlp_ubg += [cd.inf] 

        # Cumulate stage cost
        nlp_J += obj_input(Uk, Pk)
        nlp_J += obj_wp(Xk, Pk)
        nlp_J += obj_coll(Xk, Pk)
        nlp_J += obj_slack(Sk, Pk)

        Xk = Xk_next
        Uk = Uk_next

    prob = {'f': nlp_J, 'x': cd.vertcat(*nlp_x), 'g': cd.vertcat(*nlp_g), 'p': cd.vertcat(*nlp_p)}
    opts = {'print_time': 0, 'ipopt.print_level': 0, 'ipopt.max_iter': 100}
    nlp_solver = cd.nlpsol('solver', 'ipopt', prob, opts)

    return nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg
