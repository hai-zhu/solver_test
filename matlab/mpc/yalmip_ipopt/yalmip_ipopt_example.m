%% an example to use Yalmip + Ipopt to solve the NMPC problem for collision avoidance
close all
clear all
clear 
clc 


%% problem setup
problem_setup;


%% MPC formulation using Yalmip 
global model
% number of variables
model.dt   	=   dt;     % sampling time
model.N    	=   N;      % horizon length
model.nvar 	=   7;      % number of variables in z vector
                        % z = [u_x, u_y, s, x, y, v_x, v_y]'
model.neq  	=   4;      % number of equality constraints (x vector)
                        % x = [x, y, v_x, v_y]'
model.nin  	=   2;      % number of control inputs
model.nslack=   1;      % number of slack variables
model.npar 	=   19;     % number of parameters on each stage
                        % robot state (4) + robot_start, goal (4) ...
                        % + robot size (2) + mpc_weights (4) ...
                        % + obs_size (2) + obs_scale (1) + obs_pos(2) = 19
model.xl    =   [pr.ws_x(1); pr.ws_y(1); -pr.robot_maxVx; -pr.robot_maxVy];
model.xu    =   [pr.ws_x(2); pr.ws_y(2);  pr.robot_maxVx;  pr.robot_maxVy];
model.ul    =   [-pr.robot_maxAx; -pr.robot_maxAy];
model.uu    =   [ pr.robot_maxAx;  pr.robot_maxAy];
model.sl    =   0;
model.su    =   Inf;


%% simulation loop
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
robot_pos_current       = robot_pos_start;
robot_vel_current       = [0; 0];
robot_input_current     = [0; 0];
robot_z_current         = zeros(model.nvar, 1);
robot_z_current(index.z.pos) = robot_pos_current;
robot_mpc_ZPlan         = repmat(robot_z_current, 1, model.N);
% pause;
while n_loop <= max_n_loop && flag_robot_reach == 0
    flag_robot_collision = 0;
    n_loop = n_loop + 1;
    % setting real time parameters for MPC
    parameters_all_stage = zeros(model.npar, model.N);  % all paraterms on each stage
    for iStage = 1 : model.N
        parameters_all_stage(index.p.robot_state, iStage) = [robot_pos_current; robot_vel_current];
        parameters_all_stage(index.p.robot_start, iStage) = robot_pos_current;
        parameters_all_stage(index.p.robot_goal, iStage) = robot_pos_goal;
        parameters_all_stage(index.p.robot_size, iStage) = robot_size;
        parameters_all_stage(index.p.mpc_weights, iStage) = [0.0; w_inputs; w_coll; w_slack];
        if iStage == model.N    % terminal weights
            parameters_all_stage(index.p.mpc_weights, iStage) = [w_pos; w_inputs; w_coll; w_slack];
        end
        parameters_all_stage(index.p.obs_pos, iStage) = obs_pos;
        parameters_all_stage(index.p.obs_size, iStage) = obs_size;
        parameters_all_stage(index.p.obs_scale, iStage) = obs_scale;
    end
    problem.parameters_all_stage = parameters_all_stage;
    % initial conditions
    problem.xinit = [robot_pos_current; robot_vel_current];
    % solver options
    options = sdpsettings('solver','ipopt','verbose',-2,'warning',1);
    options.ipopt.print_level = 0;
    options.ipopt.tol = 1E-6;
    options.ipopt.max_iter = 300;
    % call the solver
    [mpc_output, mpc_exitflag, mpc_info] = yalmip_ipopt_solve_mpc(model, problem, options);
    mpc_solve_time = 1000*(mpc_info.solvetime+mpc_info.yalmiptime);
    % control input
    if mpc_exitflag == 1    % feasible
        mpc_infeasible = 0;
        robot_mpc_ZPlan = mpc_output.z_plan;
        robot_input_current = robot_mpc_ZPlan(index.z.inputs, 1);
    else                    % infeasible
        mpc_infeasible = 1;
        robot_input_current = -pr.robot_maxAx ...
            * robot_vel_current / max(norm(robot_vel_current),0.1);
    end
    % simulate one step
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
        fprintf('Looping [%d], MPC solve time: %.2f ms \n', ...
            n_loop, mpc_solve_time);
    end
    % update figure
    [X_temp, Y_temp] = ellipse(robot_pos_current, robot_size, 0);
    set(fig_robot_pos, 'XData', X_temp, 'YData', Y_temp); 
    set(fig_robot_mpc_path, 'XData', robot_mpc_ZPlan(index.z.pos(1), :), ...
        'YData', robot_mpc_ZPlan(index.z.pos(2), :));
    drawnow limitrate
    pause(0.05);
end
