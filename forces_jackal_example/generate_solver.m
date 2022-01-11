%% Generating a Forces Pro solver

%% problem setup
problem_setup;

%% model
global model
% number of variables
model.dt   	=   pr.dt;     % sampling time
model.N    	=   pr.N;      % horizon length
model.nvar 	=   pr.nvar;   % number of variables in z vector
model.neq  	=   pr.neq;    % number of equality constraints (x vector)
model.nh  	=   pr.nh;     % number of inequality constraints
model.nin  	=   pr.nu;     % number of control inputs
model.nslack=   pr.ns;     % number of slack variables
model.npar 	=   pr.nparam; % number of parameters on each stage
% upper/lower bound of z vector
model.lb    = [-pr.robot_maxVel, -pr.robot_maxOmega, pr.ws_x(1), pr.ws_y(1), -pi];
model.ub    = [ pr.robot_maxVel,  pr.robot_maxOmega, pr.ws_x(2), pr.ws_y(2),  pi];
% dynamics/equlities
model.eq 	=   @(z) RK2(z(index.z_states), z(index.z_inputs), ...
                        pr.robot_dynamics_continuous, model.dt);
model.E 	=   [zeros(model.neq, model.nin + model.nslack), eye(model.neq)];
% stage objective
model.objective  = @(z, p) forces_pro_obj(z, p);
% terminal objective
model.objectiveN = @(z, p) forces_pro_obj(z, p);
% inequalities
model.ineq  =   @(z, p) forces_pro_ineq(z, p);
model.hl = [0];
model.hu = [+Inf];
% which variables initial conditions are imposed (x index in z)
model.xinitidx = index.z_states;


%% solver options and solver generation
solver_name = 'forces_pro_mpc_solver';
codeoptions = forces_pro_solver_option(solver_name);
fprintf('[%s] Generating new FORCES solver...\n',datestr(now,'HH:MM:SS'));
FORCES_NLP(model, codeoptions);
fprintf('[%s] FORCES solver generated OK \n',datestr(now,'HH:MM:SS'));

