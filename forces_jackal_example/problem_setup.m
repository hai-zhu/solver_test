%% probelm setup for the collision avoidance scenario
% the global pr struct should not be changed during simulation
global pr
% workspace 
pr.ws_x = [-10.0, 10.0]';       % m
pr.ws_y = [-6.0, 6.0]';         % m
% robot dynamics
pr.robot_dynamics_continuous = @jackal_dynamics_continuous;
% robot control bound
pr.robot_maxVel = 1.2;          % m/s 
pr.robot_maxOmega = 2.0;        % rad/s 
pr.robot_maxAcc = 3.0;          % m/s^2
pr.robot_maxOmegaAcc = 6.0;     % rad/s^2
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
pr.ns = 1;                  % slack dimension
pr.nvar = pr.nu + pr.ns + pr.nx;     % nb. of variables in z vector
pr.neq = pr.nx;             % nb. of equality constraints
pr.nh = 1;                  % nb. of inequality constraints
pr.nparam = 15;             % parameter dimension 
% MPC cost terms weights, can be real time param
pr.w_pos = 8.0;
pr.w_theta = 1.0;
pr.w_input = 0.05;
pr.w_coll = 0.01;
pr.w_slack = 1E4;

%% indexing
global index
% z vector
index.z_inputs = 1:2;     % 1, 2
index.z_slacks = 3;       % 3
index.z_states = 4:6;     % 4, 5, 6 
% control vector, u = [vel, omega]
index.u_vel = 1;          % 1
index.u_omega = 2;        % 2
% slack vector
index.s_coll = 1;         % 1
% state vector, x = [px, py, theta]
index.x_pos = 1:2;        % 1, 2
index.x_theta = 3;        % 3
% param vector
index.p_robot_pos_start = 1:2;     % 1, 2
index.p_robot_pos_goal = 3:4;      % 3, 4
index.p_robot_size = 5:6;          % 5, 6
index.p_obs_pos = 7:8;             % 7, 8
index.p_obs_size = 9:10;           % 9, 10
index.p_mpc_weights = 11:15;       % 11, 12, 13, 14, 15
index.p_mpc_weights_w_pos = 1;     % 1 in mpc_weights
index.p_mpc_weights_w_yaw = 2;     % 2 in mpc_weights
index.p_mpc_weights_w_input = 3;   % 3 in mpc_weights
index.p_mpc_weights_w_coll = 4;    % 4 in mpc_weights
index.p_mpc_weights_w_slack = 5;   % 5 in mpc_weights

