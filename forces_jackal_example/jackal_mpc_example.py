import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.patches as mpatches 
from matplotlib import markers
import time 

from problem_setup import Problem, Index
from utils import my_RK2
from dynamics import jackal_dynamics_continuous

import forces_pro_mpc_solver_py

def forces_jackal_mpc_example():
    # Problem settings and indexing
    pr = Problem()
    index = Index()

    # nlp problem for the solver
    nlp_problem = {"x0": [],                # initial guess
                   "xinit": [],             # initial condition
                   "all_parameters": []}    # run time parameters

    # Prepare a figure for visualization 
    plt.ion()
    fig_main, ax_main = plt.subplots()
    ax_main.grid(visible=True, ls='-.')
    ax_main.set_aspect('equal')
    ax_main.set_xlim(pr.ws_x)
    ax_main.set_ylim(pr.ws_y)
    ax_main.set_xlabel('x [m]')
    ax_main.set_ylabel('y [m]')
    # plot obejects
    # obstalce 
    obs_pos_ellipse = mpatches.Ellipse(pr.obs_pos, 2*pr.obs_size[0], 2*pr.obs_size[1], 
                                        angle=0, fc=(1, 0, 0, 0.4), ec=(1, 0, 0, 0.2))
    fig_obs_pos = ax_main.add_artist(obs_pos_ellipse)
    # robot current pos
    robot_pos_ellipse = mpatches.Ellipse(pr.robot_pos_start, 2*pr.robot_size[0], 2*pr.robot_size[1], 
                                        angle=np.rad2deg(pr.robot_theta_start[0]), fc=(0, 0, 1, 0.8), ec=(0, 0, 1, 0.8))
    fig_robot_pos = ax_main.add_artist(robot_pos_ellipse)
    # robot goal location 
    fig_robot_goal = ax_main.plot(pr.robot_pos_goal[0], pr.robot_pos_goal[1], marker='d', mec='r', mfc='r', ms=10)
    # robot planner path 
    fig_robot_mpc_path = ax_main.plot(pr.robot_pos_start[0], pr.robot_pos_start[1], c='c', ls='-', lw=2.0)
    plt.draw()

    # Main loop
    # flags 
    mpc_feasible = False
    n_loop = 0              # nb. of loops performed
    max_n_loop = 1000       # max nb. of loops 
    robot_state_current = pr.robot_pos_start + pr.robot_theta_start    # list, [px, py, theta]
    robot_control_current = list(np.zeros(pr.nu))
    robot_z_current = robot_control_current + robot_state_current      # list, [vel, omega, px, py, theta]
    mpc_z_plan = np.tile(np.array(robot_z_current).reshape((-1, 1)), (1, pr.N))
    # loop
    while n_loop <= max_n_loop:
        n_loop += 1
        # Set initial condition 
        nlp_problem["xinit"] = np.array(robot_state_current)
        # Set real time paraterms
        param_all_stage = np.zeros((pr.nparam, pr.N))
        for iStage in range(0, pr.N):
            param_all_stage[index.p_robot_pos_start, iStage] = np.array(robot_state_current[index.x_pos])
            param_all_stage[index.p_robot_pos_goal, iStage] = np.array(pr.robot_pos_goal)
            param_all_stage[index.p_robot_size, iStage] = np.array(pr.robot_size)
            param_all_stage[index.p_obs_pos, iStage] = np.array(pr.obs_pos)
            param_all_stage[index.p_obs_size, iStage] = np.array(pr.obs_size)
            param_all_stage[index.p_mpc_weights, iStage] = np.array([0.2*pr.w_pos, pr.w_input, pr.w_coll])
            if iStage == pr.N-1:
                param_all_stage[index.p_mpc_weights, iStage] = np.array([pr.w_pos, pr.w_input, pr.w_coll])
        nlp_problem["all_parameters"] = param_all_stage.reshape((-1, 1), order='F')
        # Set initial guess
        if mpc_feasible:        # MPC feasible
            z_traj_init = np.concatenate((mpc_z_plan[:, 1:], mpc_z_plan[:, -1:]), axis=1)
        else:                   # MPC infeasible
            z_traj_init = np.tile(np.array(robot_state_current+list(np.zeros(pr.nu))).reshape((-1, 1)), (1, pr.N))
        nlp_problem["x0"] = z_traj_init.reshape((-1, 1), order='F')
        # Call the solver
        mpc_output, mpc_exitflag, mpc_info = forces_pro_mpc_solver_py.solve(nlp_problem)
        # Obtain solution
        for iStage in range(0, pr.N):
            mpc_z_plan[:, iStage] = mpc_output['x{0:02d}'.format(iStage + 1)]
        if mpc_exitflag != 1:     # infeasible
            print("FORCESPRO took {} iterations and {} seconds to solve the problem.\n".format(mpc_info.it, mpc_info.solvetime))
            mpc_feasible = False
            robot_control_current = list(0.1*mpc_z_plan[index.z_inputs, 0])
        else:               # feasible
            mpc_feasible = True
            robot_control_current = list(mpc_z_plan[index.z_inputs, 0])
        # Executing the control input 
        # in simulation via RK 
        robot_state_next = my_RK2(robot_state_current, robot_control_current, jackal_dynamics_continuous, pr.dt, [])
        # Update the system 
        robot_state_current = robot_state_next
        # Update visualization 
        fig_robot_pos.set_center(robot_state_current[index.x_pos])
        fig_robot_pos.set_angle(np.rad2deg(robot_state_current[index.x_theta]))
        mpc_x_plan = mpc_z_plan[index.z_states, :];
        fig_robot_mpc_path[0].set_data(mpc_x_plan[index.x_pos, 1:])
        fig_main.canvas.draw()
        fig_main.canvas.flush_events()
        time.sleep(0.01)
        
    
    plt.ioff()
    plt.show()

if __name__ == '__main__':
    forces_jackal_mpc_example()
    