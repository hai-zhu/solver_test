# an example to use CasADi + Ipopt, and collocation to solve the NMPC problem for collision avoidance
import warnings
import numpy as np
import casadi as cd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time

import sys
import os
from pathlib import Path

FILE_THIS = Path(__file__).resolve()
PARENT = FILE_THIS.parent
GPARENT = FILE_THIS.parents[1]
GGPARENT = FILE_THIS.parents[2]
sys.path.append(str(PARENT))
sys.path.append(str(GPARENT))
sys.path.append(str(GGPARENT))

from python_impl.objectives import obj_mpc
import python_impl.problem_setup as problem_setup
from python_impl.dynamics import double_integrator_dynamics_continuous as di
from python_impl.utils import integrator


def casadi_collocation_nlp_form(model, index, pr):
    # Define stage variables
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
    p_rob_goal = p_stage[index.p_robot_goal]
    p_rob_size = p_stage[index.p_robot_size]
    p_mpc_weights = p_stage[index.p_mpc_weights]
    p_obs_pos = p_stage[index.p_obs_pos]
    p_obs_size = p_stage[index.p_obs_size]
    p_obs_scale = p_stage[index.p_obs_scale]
    # mpc weights
    w_pos = p_mpc_weights[0]
    w_inputs = p_mpc_weights[1]
    w_coll = p_mpc_weights[2]
    w_slack = p_mpc_weights[3]

    # Define model equations
    px_dot = vx
    py_dot = vy
    vx_dot = ax
    vy_dot = ay
    x_stage_dot = cd.vertcat(px_dot, py_dot, vx_dot, vy_dot)
    dyn_f = cd.Function('dyn_f', [x_stage, u_stage], [x_stage_dot])

    # Define objective function
    # control input cost
    cost_input_acc = w_inputs * obj_mpc.obj_input_acc(u_stage, pr)
    obj_input = cd.Function('obj_input', [u_stage, p_stage], [cost_input_acc])
    # waypoint navigation cost
    cost_wp_pos = w_pos * obj_mpc.obj_desired_pos(x_stage[index.x_pos], p_rob_start, p_rob_goal)
    obj_wp = cd.Function('obj_wp', [x_stage, p_stage], [cost_wp_pos])
    # collision potentials cost
    cost_coll = w_coll * obj_mpc.obj_collision_potential(x_stage[index.x_pos], p_rob_size, p_obs_pos, p_obs_size,
                                                         p_obs_scale)
    obj_coll = cd.Function('obj_coll', [x_stage, p_stage], [cost_coll])
    # slack cost
    cost_slack = w_slack * s_stage ** 2
    obj_slack = cd.Function('obj_slack', [s_stage, p_stage], [cost_slack])

    # Define inequality constraints function
    a = p_rob_size[0] + 1.05 * p_obs_size[0]
    b = p_rob_size[1] + 1.05 * p_obs_size[1]
    d = x_stage[index.x_pos] - p_obs_pos
    ego_obs_dis = d[0] ** 2 / a ** 2 + d[1] ** 2 / b ** 2 - 1 + s_stage  # ego_obs_dis > 0
    h_ego_obs_dis = cd.Function('h_ego_obs_dis', [x_stage, p_stage, s_stage], [ego_obs_dis])

    # Definitions for collocation
    DT = model.dt
    XK = cd.MX.sym('XK', model.nx)
    XK1 = cd.MX.sym('XK1', model.nx)
    XDOTK = cd.MX.sym('XDOTK', model.nx)
    XDOTK1 = cd.MX.sym('XDOTK1', model.nx)
    UK = cd.MX.sym('UK', model.nu)
    UK1 = cd.MX.sym('UK1', model.nu)
    X_tc = 1. / 2 * (XK + XK1) + DT / 8 * (XDOTK - XDOTK1)
    f_x_tc = cd.Function('f_x_tc', [XK, XK1, XDOTK, XDOTK1], [X_tc], ['xk', 'xk1', 'xdk', 'xdk1'], ['xtc'])
    Xdot_tc = -3 / (2 * DT) * (XK - XK1) - 1. / 4 * (XDOTK + XDOTK1)
    f_xdot_tc = cd.Function('f_xdot_tc', [XK, XK1, XDOTK, XDOTK1], [Xdot_tc], ['xk', 'xk1', 'xdk', 'xdk1'], ['xdtc'])
    U_tc = 1 / 2 * (UK + UK1)
    f_u_tc = cd.Function('f_u_tc', [UK, UK1], [U_tc], ['uk', 'uk1'], ['utc'])

    # NLP Formulation
    nlp_x = []  # decision variables
    nlp_x0 = []  # initial guess
    nlp_lbx = []  # lower bound on decision variables
    nlp_ubx = []  # upper bound on decision variables
    nlp_J = 0  # cost accumulator
    nlp_g = []  # inequality constraint equations
    nlp_lbg = []  # lower bound for g
    nlp_ubg = []  # upper bound for g
    nlp_p = []  # real time parameters

    # Lift initial conditions
    Xk = cd.MX.sym('X0', model.nx)
    nlp_x += [Xk]
    nlp_lbx += [0] * model.nx
    nlp_ubx += [0] * model.nx
    nlp_x0 += [0] * model.nx

    Uk = cd.MX.sym('U0', model.nu)
    nlp_x += [Uk]
    nlp_lbx += model.ul
    nlp_ubx += model.uu
    nlp_x0 += [0] * model.nu

    # Collocation
    for k in range(model.N):
        # Parameters
        Pk = cd.MX.sym('P_' + str(k), model.np)  # parameter of this stage
        nlp_p += [Pk]

        # New NLP variables for the state
        Xk_next = cd.MX.sym('X_' + str(k + 1), model.nx)
        nlp_x += [Xk_next]
        nlp_lbx += model.xl
        nlp_ubx += model.xu
        nlp_x0 += [0] * model.nx

        # New NLP variables for the control
        Uk_next = cd.MX.sym('U_' + str(k + 1), model.nu)
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
        Sk = cd.MX.sym('S_' + str(k + 1), model.ns)
        nlp_x += [Sk]  # adding sk as decision variable
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


def casadi_collocation_example():
    recompile = True
    solver_build_name = 'casadi_collocation_nlp'

    # problem setup
    pr = problem_setup.Problem()

    # index
    index = problem_setup.Index()
    # x vector
    index.x_pos = slice(0, 2)  # 0, 1
    index.x_vel = slice(2, 4)  # 2, 3
    # p vector
    index.p_robot_start = slice(0, 2)  # 0, 1
    index.p_robot_goal = slice(2, 4)  # 2, 3
    index.p_robot_size = slice(4, 6)  # 4, 5
    index.p_mpc_weights = slice(6, 10)  # 6, 7, 8, 9
    index.p_obs_pos = slice(10, 12)  # 10, 11
    index.p_obs_size = slice(12, 14)  # 12, 13
    index.p_obs_scale = slice(14, 15)  # 14

    # MPC model
    model = problem_setup.Mpc_model()
    # MPC
    model.dt = pr.dt  # sampling time
    model.N = pr.N  # horizon length
    model.nx = 4  # number of state x = [x, y, v_x, v_y]'
    model.nu = 2  # number of control inputs, u = [a_x, a_y]'
    model.ns = 1  # number of slack variables
    model.np = 15  # number of parameters on each stage
    # robot_start, goal (4) ...
    # + robot size (2) + mpc_weights (4) ...
    # + obs_size (2) + obs_scale (1) + obs_pos(2) = 15
    # bound
    model.xl = [pr.ws_x[0], pr.ws_y[0], -pr.robot_maxVx, -pr.robot_maxVy]
    model.xu = [pr.ws_x[1], pr.ws_y[1], pr.robot_maxVx, pr.robot_maxVy]
    model.ul = [-pr.robot_maxAx, -pr.robot_maxAy]
    model.uu = [pr.robot_maxAx, pr.robot_maxAy]
    model.sl = [0.0]
    model.su = [10.0]

    # NLP formulation
    [nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg] = casadi_collocation_nlp_form(model, index, pr)
    # compiling
    if recompile:
        solver_build_c = solver_build_name + '.c'
        solver_build_o = solver_build_name + '.so'
        nlp_solver.generate_dependencies(solver_build_c)
        print('Compiling...')
        os.system('gcc -fPIC -shared ' + solver_build_c + ' -o ' + solver_build_o)
        print('Done Compiling!')
    solver_comp = cd.nlpsol('solver', 'ipopt', \
                            './' + solver_build_o, {'print_time': 0, 'ipopt.print_level': 0})

    # prepare a figure
    plt.ion()
    fig_main, ax_main = plt.subplots()
    ax_main.grid(b=True, ls='-.')
    ax_main.set_aspect('equal')
    ax_main.set_xlim(pr.ws_x)
    ax_main.set_ylim(pr.ws_y)
    ax_main.set_xlabel('x [m]')
    ax_main.set_ylabel('y [m]')
    # plotting objects
    # obstacle
    obs_ell = mpatches.Ellipse(pr.obs_pos, 2 * pr.obs_size[0], 2 * pr.obs_size[1], angle=0, fc=(1, 0, 0, 0.4),
                               ec=(1, 0, 0, 0.2))
    fig_obs_pos = ax_main.add_artist(obs_ell)
    # robot current and goal locations
    robot_pos_ell = mpatches.Ellipse(pr.robot_pos_start, 2 * pr.robot_size[0], 2 * pr.robot_size[1], angle=0,
                                     fc=(0, 0, 1, 0.8), ec=(0, 0, 1, 0.8))
    fig_robot_pos = ax_main.add_artist(robot_pos_ell)
    fig_robot_goal = ax_main.plot(pr.robot_pos_goal[0], pr.robot_pos_goal[1], marker='d', mec='r', mfc='r', ms=10)
    # robot mpc planned path
    fig_robot_mpc_path = ax_main.plot(pr.robot_pos_start[0], pr.robot_pos_start[1], c='c', ls='-', lw=2.0)
    plt.draw()

    # Main loop
    # flags
    flag_robot_reach = 0  # if robot reaching goal
    flag_robot_collision = 0  # if robot in a collision
    Tol_robot_pos = 0.1  # pos tol to determine if reaching
    Tol_robot_vel = 0.02  # vel tol
    mpc_infeasible = 1  # flag
    # simulation variables
    n_loop = 0  # number of loops performed
    max_n_loop = 1000
    mpc_solve_it = 0
    mpc_solve_time = 0.0
    mpc_solve_time_all = np.array([])
    robot_pos_current = pr.robot_pos_start  # list
    robot_vel_current = [0.0, 0.0]  # list
    robot_input_current = [0.0, 0.0]
    mpc_u_plan = np.zeros((model.nu, model.N + 1))  # numpy array
    mpc_s_plan = np.zeros((model.ns, model.N))
    mpc_x_plan = np.tile(np.concatenate((robot_pos_current, robot_vel_current)).reshape((-1, 1)), (1, model.N + 1))
    # input("Press the <ENTER> key to continue...")
    while n_loop <= max_n_loop and flag_robot_reach == 0:
        flag_robot_collision = 0
        n_loop += 1
        # prepare real-time parameters for MPC
        parameters_all_stage = np.zeros((model.np, model.N))  # all parameters on each stage
        for iStage in range(0, model.N):
            parameters_all_stage[index.p_robot_start, iStage] = robot_pos_current
            parameters_all_stage[index.p_robot_goal, iStage] = pr.robot_pos_goal
            parameters_all_stage[index.p_robot_size, iStage] = pr.robot_size
            parameters_all_stage[index.p_mpc_weights, iStage] = np.array([0.0, pr.w_inputs, pr.w_coll, pr.w_slack])
            if iStage == model.N - 1:  # terminal weights
                parameters_all_stage[index.p_mpc_weights, iStage] = np.array(
                    [pr.w_pos, pr.w_inputs, pr.w_coll, pr.w_slack])
            parameters_all_stage[index.p_obs_pos, iStage] = pr.obs_pos
            parameters_all_stage[index.p_obs_size, iStage] = pr.obs_size
            parameters_all_stage[index.p_obs_scale, iStage] = pr.obs_scale
        # set parameters
        nlp_p = np.transpose(parameters_all_stage).reshape(-1)
        # setting initial value via lbx and ubx
        nlp_lbx[index.x_pos] = robot_pos_current
        nlp_lbx[index.x_vel] = robot_vel_current
        nlp_ubx[index.x_pos] = robot_pos_current
        nlp_ubx[index.x_vel] = robot_vel_current
        # set initial guess
        nlp_x0 = []
        nlp_x0 += robot_pos_current
        nlp_x0 += robot_vel_current
        if mpc_infeasible == 0:  # if feasible
            nlp_x0 += list(mpc_u_plan[:, 1])
            for iTemp in range(0, model.N - 1):
                x_u_s_temp = list(mpc_x_plan[:, iTemp + 1]) + list(mpc_u_plan[:, iTemp + 1]) + [0] * model.ns
                nlp_x0 += x_u_s_temp
            nlp_x0 += x_u_s_temp  # duplicate the last one
        else:  # infeasible
            nlp_x0 += [0] * model.nu
            x_u_s_temp = robot_pos_current + robot_vel_current + [0] * model.nu + [0] * model.ns
            nlp_x0 += x_u_s_temp * model.N
        # call the solver
        tic = time.time()
        sol = solver_comp(x0=nlp_x0, p=nlp_p, lbx=nlp_lbx, ubx=nlp_ubx, lbg=nlp_lbg, ubg=nlp_ubg)
        mpc_solve_time = time.time() - tic
        mpc_solve_time_all = np.append(mpc_solve_time_all, mpc_solve_time)
        # retrive solution
        x_opt = sol['x']
        mpc_x_plan[:, 0:1] = np.array(x_opt[0: model.nx]).reshape(model.nx, 1)
        mpc_u_plan[:, 0:1] = x_opt[model.nx: model.nx + model.nu]
        for i in range(1, model.N + 1):  # N future stages, 1,..,N
            x_u_s_temp = x_opt[model.nx + model.nu + (model.nu + model.nx + model.ns) * (i - 1): \
                               model.nx + model.nu + (model.nu + model.nx + model.ns) * (i)]
            mpc_x_plan[:, i:i + 1] = x_u_s_temp[0: model.nx]
            mpc_u_plan[:, i:i + 1] = x_u_s_temp[model.nx: model.nx + model.nu]
            mpc_s_plan[:, i - 1] = x_u_s_temp[model.nu + model.nx: model.nu + model.nx + model.ns]
        robot_input_current = mpc_u_plan[:, 0]
        # simulate one step
        # robot_pos_current = list(mpc_x_plan[index.x_pos, 1])    # for test
        # robot_vel_current = list(mpc_x_plan[index.x_vel, 1])
        x_now = robot_pos_current + robot_vel_current  # list append
        x_next = integrator.my_RK2(x_now, robot_input_current, di.double_integrator_dynamics_continuous, model.dt, [])
        robot_pos_current = x_next[index.x_pos]
        robot_vel_current = x_next[index.x_vel]
        # determine if reaching
        robot_pos_rel = np.array(robot_pos_current) - np.array(pr.robot_pos_goal)
        if np.linalg.norm(robot_pos_rel) < Tol_robot_pos and np.linalg.norm(
                np.array(robot_vel_current)) < Tol_robot_vel:
            flag_robot_reach = 1
            print('Robot arrived! \n')
        # collision checking
        d_vec_obs = np.array(robot_pos_current) - np.array(pr.obs_pos)
        coll_size = np.array(pr.robot_size) + np.array(pr.obs_size)
        if np.sqrt(d_vec_obs[0] ** 2 / coll_size[0] ** 2 + d_vec_obs[1] ** 2 / coll_size[1] ** 2 - 1) <= 0:
            flag_robot_collision = 1
            warnings.showwarning('Collision happens!')
        # printing
        if np.mod(n_loop, 10) == 1:
            print('loop', n_loop, 'mpc solver time: ', mpc_solve_time * 1000, 'ms')
        # update figure
        fig_robot_pos.set_center(robot_pos_current)
        fig_robot_mpc_path[0].set_data(mpc_x_plan[0:2, 1:])
        fig_main.canvas.draw()
        fig_main.canvas.flush_events()
        # time.sleep(model.dt)
        time.sleep(0.01)

    plt.ioff()
    plt.show()


if __name__ == '__main__':
    casadi_collocation_example()
