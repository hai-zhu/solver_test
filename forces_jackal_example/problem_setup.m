%% probelm setup for the collision avoidance scenario
% the global pr struct should not be changed during simulation
global pr
% workspace 
pr.ws_x = [-10.0, 10.0]';       % m
pr.ws_y = [-6.0, 6.0]';         % m
% robot dynamics
pr.robot_dynamics_continuous = @jackal_dynamics_continuous;
% robot control bound
pr.robot_maxVel = 1.0;          % m/s 
pr.robot_maxOmega = deg2rad(15.0);      % rad/s 
% robot size, start and goal positions, can be real time param
pr.robot_size = [0.5, 0.3]';    % ellipse, m
pr.robot_pos_start = [-8.0, 0.0]';      % m
pr.robot_theta_start = deg2rad(0.0);    % rad
pr.robot_pos_goal = [8.0, 0.0]';        % m
% obstacle size, position, can be real time param 
pr.obs_size = [1.6, 1.0]';      % m
pr.obs_pos = [0.0, -0.1]';      % m
% MPC settings
pr.dt = 0.1;                % sampling time, s
pr.N = 20;                  % horizon length
pr.T = pr.N*pr.dt;          % horizon time
pr.nx = 3;                  % state dimension 
pr.nu = 2;                  % control dimension
pr.ns = 0;                  % slack dimension
pr.nvar = pr.nu + pr.ns + pr.nx;     % nb. of variables in z vector
pr.neq = pr.nx;             % nb. of equality constraints
pr.nh = 1;                  % nb. of inequality constraints
pr.nparam = 13;             % parameter dimension 
% MPC cost terms weights, can be real time param
pr.w_pos = 8.0;
pr.w_input = 0.05;
pr.w_coll = 0.01;

%% indexing
global index
% state vector, x = [px, py, theta]
index.x_pos = 1:2;        % 1, 2
index.x_theta = 3;        % 3
% control vector, u = [vel, omega]
index.u_vel = 1;          % 0
index.u_omega = 2;        % 2
% z vector
index.z_inputs = 1:2;     % 1, 2
index.z_states = 3:5;     % 3, 4, 5
% param vector
index.p_robot_pos_start = 1:2;     % 1, 2
index.p_robot_pos_goal = 3:4;      % 3, 4
index.p_robot_size = 5:6;          % 5, 6
index.p_obs_pos = 7:8;             % 7, 8
index.p_obs_size = 9:10;           % 9, 10
index.p_mpc_weights = 11:13;       % 11, 12, 13
index.p_mpc_weights_w_pos = 1;     % 1 in mpc_weights
index.p_mpc_weights_w_input = 2;   % 2 in mpc_weights
index.p_mpc_weights_w_coll = 3;    % 3 in mpc_weights

