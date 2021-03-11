%% an example to use CasADi and multiple shooting to solve the NMPC problem 
% for collision avoidance
close all
clear all
clear 
clc 

import casadi.*         % to use CasADi sytax

%% problem setup
problem_setup;
recompile = true;
solver_build_name = 'casadi_collocation_nlp';

%% MPC formulation to use CasADi
global model
model.robot_dynamics_continuous = pr.robot_dynamics_continuous;
% number of variables
model.dt   	=   dt;     % sampling time
model.N    	=   N;      % total horizon length
model.nx  	=   4;      % number of states x = [x, y, v_x, v_y]'
model.nu  	=   2;      % number of control inputs, u = [a_x, a_y]'
model.ns    =   1;      % number of slack variables
model.np 	=   15;     % number of parameters on each stage
                        % robot_start, goal (4) ...
                        % + robot size (2) + mpc_weights (4) ...
                        % + obs_size (2) + obs_scale (1) + obs_pos(2) = 15
% bounds
model.xl    =   [pr.ws_x(1); pr.ws_y(1); -pr.robot_maxVx; -pr.robot_maxVy];
model.xu    =   [pr.ws_x(2); pr.ws_y(2);  pr.robot_maxVx;  pr.robot_maxVy];
model.ul    =   [-pr.robot_maxAx; -pr.robot_maxAy];
model.uu    =   [ pr.robot_maxAx;  pr.robot_maxAy];
% index
global index
% x vector
index.x.pos     =   1:2;
index.x.vel     =   3:4;
% p vector
index.p.robot_start =   1:2;
index.p.robot_goal  =   3:4;
index.p.robot_size  =   5:6;
index.p.mpc_weights =   7:10;
index.p.obs_pos     =   11:12;
index.p.obs_size    =   13:14;
index.p.obs_scale   =   15;


%% NLP formulation and solver
[nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg] = ...
    casadi_collocation_nlp_form(model);
if recompile
     % generate, generate_dependencies
    gen_opts = struct('mex', true);
    nlp_solver.generate_dependencies([solver_build_name, '.c'], gen_opts); 
    disp('Compiling...');
    system(['gcc -fPIC -shared ', solver_build_name, '.c', ...
        ' -o ', solver_build_name, '.so']);
    disp('Done Compiling!');
    % move file
    folder_name = './matlab/mpc/casadi_collocation/build/';
    mkdir(folder_name);
    rmdir(folder_name, 's');
    mkdir(folder_name);
    movefile([solver_build_name, '*'], folder_name);  % move the files
end
opts = struct();                % solver options
opts.print_time = 0;
opts.ipopt.print_level = 0;
opts.ipopt.max_iter = 100;
opts.ipopt.tol = 1E-4;
solver_comp = nlpsol('solver', 'ipopt', [folder_name, solver_build_name, '.so'], opts);

%% simulation loop
%% plooting figure
fig_main = figure;              % main figure
hold on;
box on; 
grid on;
axis equal;
axis([pr.ws_x', pr.ws_y']);
xlabel('x [m]');
ylabel('y [m]');
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
mpc_infeasible          = 1;      	% flag
% simulation variables
n_loop = 0;                         % number of loops performed
max_n_loop = 1000;
mpc_solve_it = 0;
mpc_solve_time = 0;
mpc_solve_time_all = [];
robot_pos_current       = robot_pos_start;
robot_vel_current       = [0; 0];
robot_input_current     = [0; 0];
mpc_x_plan = repmat([robot_pos_current; robot_vel_current], 1, model.N+1);
mpc_u_plan = zeros(model.nu, model.N+1);
mpc_s_plan = zeros(model.ns, model.N);
% pause;
while n_loop <= max_n_loop && flag_robot_reach == 0
    flag_robot_collision = 0;
    n_loop = n_loop + 1;
    % setting real time parameters for MPC
    parameters_all_stage = zeros(model.np, model.N);  % all paraterms on each stage
    for iStage = 1 : model.N
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
    nlp_p = reshape(parameters_all_stage, [], 1);  % into a column vector
    % setting initial value via lbx and ubx
    nlp_lbx(index.x.pos) = robot_pos_current;
    nlp_lbx(index.x.vel) = robot_vel_current;
    nlp_ubx(index.x.pos) = robot_pos_current;
    nlp_ubx(index.x.vel) = robot_vel_current;
    % set initial guess
    if mpc_infeasible == 0      % feasible
        nlp_x0 = [robot_pos_current; robot_vel_current; mpc_u_plan(:, 2)];
        for i = 1 : model.N-1
            x_u_s_temp = [mpc_x_plan(:, i+2); mpc_u_plan(:, i+2); zeros(model.ns, 1)];
            nlp_x0 = [nlp_x0; x_u_s_temp];
        end
        nlp_x0 = [nlp_x0; x_u_s_temp];
    else                        % infeasible
        x_u_s_temp = [robot_pos_current; robot_vel_current; zeros(model.nu,1); zeros(model.ns, 1)];
        nlp_x0 = [robot_pos_current; robot_vel_current; zeros(model.nu, 1); ...
            repmat(x_u_s_temp, model.N, 1)];
    end
    % call the solver
    tic;
%     sol = nlp_solver('x0', nlp_x0, 'p', nlp_p, 'lbx', nlp_lbx, 'ubx', nlp_ubx,...
%             'lbg', nlp_lbg, 'ubg', nlp_ubg);  % not using code generation
    sol = solver_comp('x0', nlp_x0, 'p', nlp_p, 'lbx', nlp_lbx, 'ubx', nlp_ubx,...
            'lbg', nlp_lbg, 'ubg', nlp_ubg);
    mpc_solve_time = 1000*toc;
    mpc_solve_time_all = [mpc_solve_time_all; mpc_solve_time];
    % retrive solution
    x_opt = full(sol.x);
    mpc_x_plan(:, 1) = x_opt(1:model.nx);
    mpc_u_plan(:, 1) = x_opt(model.nx+1 : model.nx+model.nu);
    for i = 1 : model.N
        x_u_s_temp = x_opt(model.nx+model.nu+(model.nu+model.nx+model.ns)*(i-1)+1 : ...
            model.nx+model.nu+(model.nu+model.nx+model.ns)*(i));
        mpc_x_plan(:, i+1) = x_u_s_temp(1 : model.nx);
        mpc_u_plan(:, i) = x_u_s_temp(model.nx+1 : model.nx+model.nu);
        mpc_s_plan(:, i) = x_u_s_temp(model.nu+model.nx+1 : model.nu+model.nx+model.ns);
    end
    robot_input_current = mpc_u_plan(:, 1);
    % simulate one step
%     robot_pos_current = mpc_x_plan(index.x.pos, 2);
%     robot_vel_current = mpc_x_plan(index.x.vel, 2);
    % simulate one step
    x_now = [robot_pos_current; robot_vel_current];
    x_next = my_RK2(x_now, robot_input_current, pr.robot_dynamics_continuous, ...
        model.dt, []);
    robot_pos_current = x_next(index.x.pos);
    robot_vel_current = x_next(index.x.vel);
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
        fprintf('Looping [%d], MPC solve time: %.2f ms, MPC solve iterationns: %d \n', ...
            n_loop, mpc_solve_time, mpc_solve_it);
    end
    % update figure
    [X_temp, Y_temp] = ellipse(robot_pos_current, robot_size, 0);
    set(fig_robot_pos, 'XData', X_temp, 'YData', Y_temp); 
    set(fig_robot_mpc_path, 'XData', mpc_x_plan(1, 2:end), ...
        'YData', mpc_x_plan(2, 2:end));
    drawnow limitrate
    pause(0.05);
end
mpc_solve_time_avg = mean(mpc_solve_time_all)
