%% an example to use acados to solve the NMPC problem for collision avoidance
close all
clear all
clear 
clc 

import casadi.*                     % to use CasADi sytax


%% set path
addpath(genpath([pwd, '/../../../utils']));             % utils 
addpath(genpath([pwd, '/../../../matlab_impl/dynamics']));   % system dynamics
addpath(genpath([pwd, '/../../../matlab_impl/objectives'])); % MPC objectives


%% problem setup
pr.name     =   'di_mpc_ca';    % ocp name
pr.output_dir = './build';      % code building directory
pr.new_solver = 1;              % if regenerate the solver
% workspace environment
pr.ws_x    	= [-10; 10];
pr.ws_y   	= [-6; 6];
% robot physics
pr.robot_dynamics_continuous  = @double_integrator_dynamics_continuous;
pr.robot_maxVx  = 2.0;  % velocity bound
pr.robot_maxVy  = 2.0;
pr.robot_maxAx  = 1.0;  % control (acceleration bound)
pr.robot_maxAy  = 1.0;
% mpc settings
pr.dt       =   0.1; 	% sampling time
pr.N        =   20;     % nb of stages
pr.T        =   pr.dt * pr.N;   % time horizon length
pr.neq      =   4;  	% nb of states variables, x = [x, y, v_x, v_y]'
pr.nin      =   2;      % nb of control variables, u = [a_x, a_y]'
pr.nh       =   1;      % nb of inequality constraints
pr.npar     =   15;     % nb of of parameters on each stage
                        % robot_start, goal (4) ...
                        % + robot size (2) + mpc_weights (4) ...
                        % + obs_size (2) + obs_scale (1) + obs_pos(2) = 19
% index in parameter vector
pr.index.p.robot_start =   1:2;
pr.index.p.robot_goal  =   3:4;
pr.index.p.robot_size  =   5:6;
pr.index.p.mpc_weights =   7:10;
pr.index.p.obs_pos     =   11:12;
pr.index.p.obs_size    =   13:14;
pr.index.p.obs_scale   =   15;


%% system and ocp model, solver options
model = acados_di_ca_model(pr);         % dynamics, constraints, costs
ocp_model = acados_mpc_form(model);     % formulating acados ocp problem
ocp_opts = acados_mpc_options(model);   % solver options


%% create ocp solver
ocp = acados_ocp(ocp_model, ocp_opts);  % compile and generate the solver


%% simulation loop
%% initial settings, the following settings (parameters) can be changed in real time
% obstacle physic
obs_size        = [1.6; 1.0];         	% [a, b]
obs_scale       = 1.6;                  % collision potential scale
% robot start and goal postion
robot_size      = [0.4; 0.4];
robot_pos_start = [-8; 0];              % 2x1
robot_pos_goal  = [ 8; 0];              % 2x1
% obstacle pos
obs_pos         = [ 0; -0.1];
% MPC parameters
dt              = 0.1;
N               = 20;
w_pos           = 8.0;                  % mpc weights
w_inputs        = 0.1;
w_coll          = 0.0;
w_slack         = 1E4;

%% plooting figure
fig_main = figure;              % main figure
hold on;
box on; 
grid on;
axis equal;
axis([pr.ws_x', pr.ws_y']);
xlabel('x [m]');
ylabel('y [m]')
ax_main = fig_main.CurrentAxes;
fig_ell_obs = plot_ellipse_2D(ax_main, obs_pos, obs_size, 0, ...
            'FaceColor', 'r', 'FaceAlpha', 0.4, ...
            'EdgeColor', 'r', 'EdgeAlpha', 0.2);
fig_robot_pos = plot_ellipse_2D(ax_main, robot_pos_start, robot_size, 0, ...
            'FaceColor', 'b', 'FaceAlpha', 0.8, ...
            'EdgeColor', 'b', 'EdgeAlpha', 0.8);
fig_robot_goal = plot(ax_main, robot_pos_goal(1), robot_pos_goal(2), ...
            'd', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', ...
            'MarkerSize', 10);
fig_robot_mpc_path = plot(ax_main, robot_pos_start(1), robot_pos_start(2), ...
            'Color', 'c', 'LineStyle', '-', 'LineWidth', 2.0);
%% main loop
% flags
flag_robot_reach        = 0;        % if robot reaching goal
flag_robot_collision    = 0;        % if robot in a collision
Tol_robot_pos           = 0.1;   	% pos tol to determine if reaching
Tol_robot_vel           = 0.02;    	% vel tol
mpc_infeasible          = 0;      	% flag
% simulation variables
n_loop = 0;                         % number of loops performed
max_n_loop = 1000;
mpc_solve_it = 0;
mpc_solve_time = 0;
mpc_solve_time_all = [];
robot_pos_current       = robot_pos_start;
robot_vel_current       = [0; 0];
robot_input_current     = [0; 0];
x_traj = repmat([robot_pos_current; robot_vel_current], 1, model.N+1);
u_traj = zeros(model.nu, model.N);
% pause;
while n_loop <= max_n_loop && flag_robot_reach == 0
    flag_robot_collision = 0;
    n_loop = n_loop + 1;
    % set initial condition
    ocp.set('constr_x0', [robot_pos_current; robot_vel_current]);
    % set real-time parameters
    for iStage = 0 : model.N
        parameters_this_stage = zeros(model.np, 1);
        parameters_this_stage(pr.index.p.robot_start) = robot_pos_current;
        parameters_this_stage(pr.index.p.robot_goal)  = robot_pos_goal;
        parameters_this_stage(pr.index.p.robot_size)  = robot_size;
        parameters_this_stage(pr.index.p.mpc_weights) = [0.0; w_inputs; w_coll; w_slack];
        if iStage == model.N        % terminal weights
            parameters_this_stage(pr.index.p.mpc_weights) = [w_pos; w_inputs; w_coll; w_slack];
        end
        parameters_this_stage(pr.index.p.obs_pos)     = obs_pos;
        parameters_this_stage(pr.index.p.obs_size)    = obs_size;
        parameters_this_stage(pr.index.p.obs_scale)   = obs_scale;
        ocp.set('p', parameters_this_stage, iStage);
    end
    % set trajectory initialization
    if mpc_infeasible == 0          % feasible
        x_traj_init = [x_traj(:,2:end), x_traj(:,end)];
        u_traj_init = [u_traj(:,2:end), u_traj(:,end)];
    else                            % infeasible
        x_traj_init = repmat([robot_pos_current; robot_vel_current], 1, model.N+1);
        u_traj_init = zeros(model.nu, model.N);
    end
    ocp.set('init_x', x_traj_init);
	ocp.set('init_u', u_traj_init);
    % solve OCP
    ocp.solve();                    % often maxi_iter for some solvers
    status = ocp.get('status');
        %statuses = {
        %    0: 'ACADOS_SUCCESS',
        %    1: 'ACADOS_FAILURE',
        %    2: 'ACADOS_MAXITER',
        %    3: 'ACADOS_MINSTEP',
        %    4: 'ACADOS_QP_FAILURE',
        %    5: 'ACADOS_READY'
    sqp_iter = ocp.get('sqp_iter');
    time_tot = ocp.get('time_tot');
    time_lin = ocp.get('time_lin');
    time_qp_sol = ocp.get('time_qp_sol'); 
    mpc_solve_time = 1000*time_tot;
    mpc_solve_time_all = [mpc_solve_time_all; mpc_solve_time];
    % get solution for initialization of next NLP
	x_traj = ocp.get('x');
	u_traj = ocp.get('u');
    % retrive control input and prepare next-step initialization
    if status == 0 || status == 2
        mpc_infeasible = 0;
        robot_input_current = u_traj(:, 1);
    else
        warning('NLP infeasible!');
        mpc_infeasible = 1;
        robot_input_current = -pr.robot_maxAx ...
            * robot_vel_current / max(norm(robot_vel_current),0.1);
    end
    % simulate one step
    x_now = [robot_pos_current; robot_vel_current];
    x_next = my_RK2(x_now, robot_input_current, pr.robot_dynamics_continuous, ...
        model.dt, []);
    robot_pos_current = x_next(1:2);
    robot_vel_current = x_next(3:4);
    % determin if reaching
    if norm(robot_pos_current-robot_pos_goal) < Tol_robot_pos && ...
            norm(robot_vel_current) < Tol_robot_vel
        flag_robot_reach = 1;
        fprintf('Robot arrived! \n');
    end
    % collison checking
    d_vec_obs = robot_pos_current - obs_pos;
    coll_size = robot_size + obs_size;
    if sqrt(d_vec_obs(1)^2/coll_size(1)^2 + d_vec_obs(2)^2/coll_size(2)^2 - 1) <= 0
        flag_robot_collision = 1;
        warning('Collision happens!');
    end
    % printing
    if(mod(n_loop, 10) == 1)
        fprintf('Looping [%d], time_tot: %.2f ms, sqp iter: %d \n', ...
            n_loop, mpc_solve_time, sqp_iter);
    end
    % update figure
    [X_temp, Y_temp] = ellipse(robot_pos_current, robot_size, 0);
    set(fig_robot_pos, 'XData', X_temp, 'YData', Y_temp); 
    set(fig_robot_mpc_path, 'XData', x_traj(1, 2:end), ...
        'YData', x_traj(2, 2:end));
    drawnow limitrate
    pause(0.05);
end
mpc_solve_time_avg = mean(mpc_solve_time_all)
