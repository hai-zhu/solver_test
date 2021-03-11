%% probelm setup for the collision avoidance scenario
% the global pr struct should not be changed during simulation
global pr
% workspace environment
pr.ws_x         = [-10; 10];
pr.ws_y         = [-6; 6];
% robot physics
pr.robot_dynamics_continuous  = @double_integrator_dynamics_continuous;	% dynamics
pr.robot_maxVx  = 2.0;
pr.robot_maxVy  = 2.0;
pr.robot_maxAx  = 1.0;
pr.robot_maxAy  = 1.0;
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
