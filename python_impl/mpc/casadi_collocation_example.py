## an example to use CasADi + Ipopt, and collocation to solve the NMPC problem for collision avoidance
import warnings
import numpy as np
import casadi as cd 
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time 
from dataclasses import dataclass

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

import problem_setup 
import casadi_collocation_nlp_form
from dynamics import double_integrator_dynamics_continuous as di 
from utils.integrator import integrator 

recompile = True     
solver_build_name = 'casadi_collocation_nlp'


## problem setup
pr = problem_setup.Problem()


## index
index = problem_setup.Index()
# x vector
index.x_pos     =   slice(0,2)          # 0, 1
index.x_vel     =   slice(2,4)          # 2, 3
# p vector
index.p_robot_start =   slice(0,2)      # 0, 1
index.p_robot_goal  =   slice(2,4)      # 2, 3
index.p_robot_size  =   slice(4,6)      # 4, 5
index.p_mpc_weights =   slice(6,10)     # 6, 7, 8, 9
index.p_obs_pos     =   slice(10,12)    # 10, 11
index.p_obs_size    =   slice(12,14)    # 12, 13
index.p_obs_scale   =   slice(14, 15)   # 14


## MPC model
model = problem_setup.Mpc_model()
# MPC
model.dt   	=   pr.dt       # sampling time
model.N    	=   pr.N        # horizon length
model.nx  	=   4           # number of state x = [x, y, v_x, v_y]'
model.nu  	=   2           # number of control inputs, u = [a_x, a_y]'
model.ns    =   1           # number of slack variables
model.np 	=   15          # number of parameters on each stage
                            # robot_start, goal (4) ...
                            # + robot size (2) + mpc_weights (4) ...
                            # + obs_size (2) + obs_scale (1) + obs_pos(2) = 15
# bound 
model.xl    =   [pr.ws_x[0], pr.ws_y[0], -pr.robot_maxVx, -pr.robot_maxVy]
model.xu    =   [pr.ws_x[1], pr.ws_y[1],  pr.robot_maxVx,  pr.robot_maxVy]
model.ul    =   [-pr.robot_maxAx, -pr.robot_maxAy]
model.uu    =   [ pr.robot_maxAx,  pr.robot_maxAy]
model.sl    =   [0.0]
model.su    =   [10.0]


## NLP formulation
[nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg] = casadi_collocation_nlp_form.casadi_collocation_nlp_form(model, index, pr)
# compiling 
if recompile:
    solver_build_c = solver_build_name + '.c'
    solver_build_o = solver_build_name + '.so'
    nlp_solver.generate_dependencies(solver_build_c)
    print('Compiling...')
    os.system('gcc -fPIC -shared ' + solver_build_c + ' -o ' + solver_build_o)
    print('Done Compiling!')
solver_comp = cd.nlpsol('solver', 'ipopt', \
    './'+solver_build_o, {'print_time': 0, 'ipopt.print_level' : 0})
    

## prepare a figure
plt.ion()
fig_main, ax_main = plt.subplots() 
ax_main.grid(b=True, ls='-.')
ax_main.set_aspect('equal')
ax_main.set_xlim(pr.ws_x)
ax_main.set_ylim(pr.ws_y)
ax_main.set_xlabel('x [m]')
ax_main.set_ylabel('y [m]')
## plotting objects
# obstacle
obs_ell = mpatches.Ellipse(pr.obs_pos, 2*pr.obs_size[0], 2*pr.obs_size[1], angle=0, fc=(1,0,0,0.4), ec=(1,0,0,0.2))
fig_obs_pos = ax_main.add_artist(obs_ell)
# robot current and goal locations
robot_pos_ell = mpatches.Ellipse(pr.robot_pos_start, 2*pr.robot_size[0], 2*pr.robot_size[1], angle=0, fc=(0,0,1,0.8), ec=(0,0,1,0.8))
fig_robot_pos = ax_main.add_artist(robot_pos_ell)
fig_robot_goal = ax_main.plot(pr.robot_pos_goal[0], pr.robot_pos_goal[1], marker='d', mec='r', mfc='r', ms=10)
# robot mpc planned path
fig_robot_mpc_path = ax_main.plot(pr.robot_pos_start[0], pr.robot_pos_start[1], c='c', ls='-', lw=2.0)
plt.draw()


## Main loop
# flags
flag_robot_reach        = 0         # if robot reaching goal
flag_robot_collision    = 0         # if robot in a collision
Tol_robot_pos           = 0.1   	# pos tol to determine if reaching
Tol_robot_vel           = 0.02    	# vel tol
mpc_infeasible          = 1      	# flag
# simulation variables
n_loop = 0                          # number of loops performed
max_n_loop = 1000
mpc_solve_it = 0
mpc_solve_time = 0.0
mpc_solve_time_all = np.array([])
robot_pos_current       = pr.robot_pos_start    # list 
robot_vel_current       = [0.0, 0.0]            # list
robot_input_current     = [0.0, 0.0]
mpc_u_plan = np.zeros((model.nu, model.N+1))    # numpy array
mpc_s_plan = np.zeros((model.ns, model.N))
mpc_x_plan = np.tile(np.concatenate((robot_pos_current, robot_vel_current)).reshape((-1, 1)), (1, model.N+1))
# input("Press the <ENTER> key to continue...")
while n_loop <= max_n_loop and flag_robot_reach == 0:
    flag_robot_collision = 0
    n_loop += 1
    # prepare real-time parameters for MPC
    parameters_all_stage = np.zeros((model.np, model.N))    # all parameters on each stage
    for iStage in range(0, model.N):
        parameters_all_stage[index.p_robot_start, iStage] = robot_pos_current 
        parameters_all_stage[index.p_robot_goal, iStage] = pr.robot_pos_goal
        parameters_all_stage[index.p_robot_size, iStage] = pr.robot_size
        parameters_all_stage[index.p_mpc_weights, iStage] = np.array([0.0, pr.w_inputs, pr.w_coll, pr.w_slack])
        if iStage == model.N-1:    # terminal weights
            parameters_all_stage[index.p_mpc_weights, iStage] = np.array([pr.w_pos, pr.w_inputs, pr.w_coll, pr.w_slack])
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
    if mpc_infeasible == 0:         # if feasible
        nlp_x0 += list(mpc_u_plan[:, 1])
        for iTemp in range(0, model.N-1):
            x_u_s_temp = list(mpc_x_plan[:, iTemp+1]) + list(mpc_u_plan[:, iTemp+1]) + [0]*model.ns
            nlp_x0 += x_u_s_temp
        nlp_x0 += x_u_s_temp        # duplicate the last one
    else:                           # infeasible 
        nlp_x0 += [0] * model.nu 
        x_u_s_temp = robot_pos_current + robot_vel_current + [0] * model.nu  + [0] * model.ns 
        nlp_x0 += x_u_s_temp * model.N 
    # call the solver
    tic = time.time()
    sol = solver_comp(x0=nlp_x0, p=nlp_p, lbx=nlp_lbx,ubx=nlp_ubx, lbg=nlp_lbg, ubg=nlp_ubg)
    mpc_solve_time = time.time() - tic
    mpc_solve_time_all = np.append(mpc_solve_time_all, mpc_solve_time)
    # retrive solution
    x_opt = sol['x']
    mpc_x_plan[:, 0:1] = np.array(x_opt[0 : model.nx]).reshape(model.nx, 1)
    mpc_u_plan[:, 0:1] = x_opt[model.nx : model.nx+model.nu]
    for i in range(1, model.N+1):           # N future stages, 1,..,N 
        x_u_s_temp = x_opt[model.nx+model.nu+(model.nu+model.nx+model.ns)*(i-1) : \
            model.nx+model.nu+(model.nu+model.nx+model.ns)*(i)]
        mpc_x_plan[:, i:i+1] = x_u_s_temp[0 : model.nx]
        mpc_u_plan[:, i:i+1] = x_u_s_temp[model.nx : model.nx+model.nu]
        mpc_s_plan[:, i-1] = x_u_s_temp[model.nu+model.nx : model.nu+model.nx+model.ns]
    robot_input_current = mpc_u_plan[:, 0]
    # simulate one step
    # robot_pos_current = list(mpc_x_plan[index.x_pos, 1])    # for test 
    # robot_vel_current = list(mpc_x_plan[index.x_vel, 1])
    x_now = robot_pos_current + robot_vel_current             # list append
    x_next = integrator.my_RK2(x_now, robot_input_current, di.double_integrator_dynamics_continuous, model.dt, [])
    robot_pos_current = x_next[index.x_pos]
    robot_vel_current = x_next[index.x_vel]
    # determine if reaching
    robot_pos_rel = np.array(robot_pos_current) - np.array(pr.robot_pos_goal)
    if np.linalg.norm(robot_pos_rel) < Tol_robot_pos and np.linalg.norm(np.array(robot_vel_current)) < Tol_robot_vel:
        flag_robot_reach = 1
        print('Robot arrived! \n')
    # collision checking
    d_vec_obs = np.array(robot_pos_current) - np.array(pr.obs_pos)
    coll_size = np.array(pr.robot_size) + np.array(pr.obs_size)
    if np.sqrt(d_vec_obs[0]**2/coll_size[0]**2 + d_vec_obs[1]**2/coll_size[1]**2 - 1) <= 0:
        flag_robot_collision = 1
        warnings.showwarning('Collision happens!')
    # printing
    if np.mod(n_loop, 10) == 1:
        print('loop', n_loop, 'mpc solver time: ', mpc_solve_time*1000, 'ms')
    # update figure
    fig_robot_pos.set_center(robot_pos_current)
    fig_robot_mpc_path[0].set_data(mpc_x_plan[0:2, 1:])
    fig_main.canvas.draw()
    fig_main.canvas.flush_events()
    # time.sleep(model.dt)
    time.sleep(0.01)


plt.ioff()
plt.show()
