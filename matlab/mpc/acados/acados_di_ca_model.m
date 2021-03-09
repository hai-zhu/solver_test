function model = acados_di_ca_model(pr)

    % model that describes the dynamics, constraints and costs of the
    % collision avoidance system
    
    % here model is used to store intermediate variables to pass to the real ocp model
    
    import casadi.*
    model = struct();
    
    %% Name and flag
    model.name = pr.name;
    model.output_dir = pr.output_dir;
    model.new_solver = pr.new_solver;
    
    %% Time horizon
    model.N  = pr.N;
    model.dt = pr.dt;
    model.T  = pr.T;
    
    %% Dimensions
    model.nx = pr.neq;          % nb of state
    model.nu = pr.nin;          % nb of control input
    model.nh = pr.nh;           % nb of inequlity constraint
    model.np = pr.npar;         % nb of parameters each stage
    
    %% Named symbolic variables
    ego_state   = SX.sym('ego_state', model.nx);
    ego_inputs  = SX.sym('ego_inputs', model.nu);
    ego_state_dot = SX.sym('ego_state_dot', model.nx);
    para        = SX.sym('para', model.np);
    % fetch para
    p_rob_start = para(pr.index.p.robot_start);
    p_rob_goal = para(pr.index.p.robot_goal);
    p_rob_size  = para(pr.index.p.robot_size);
    p_mpc_weights = para(pr.index.p.mpc_weights);
    p_obs_pos   = para(pr.index.p.obs_pos);
    p_obs_size  = para(pr.index.p.obs_size);
    p_obs_scale = para(pr.index.p.obs_scale);
    % mpc weights
    w_pos       =   p_mpc_weights(1);
    w_inputs    =   p_mpc_weights(2);
    w_coll      =   p_mpc_weights(3);
    w_slack     =   p_mpc_weights(4);
    
    %% Bounds
    % pos
    model.pos_x_min     =   pr.ws_x(1);
    model.pos_x_max     =   pr.ws_x(2);
    model.pos_y_min     =   pr.ws_y(1);
    model.pos_y_max     =   pr.ws_y(2);
    % vel
    model.vel_x_min     =   -pr.robot_maxVx;
    model.vel_x_max     =    pr.robot_maxVx;
    model.vel_y_min     =   -pr.robot_maxVy;
    model.vel_y_max     =    pr.robot_maxVy;
    % inputs
    model.acc_x_min     =   -pr.robot_maxAx;
    model.acc_x_max     =    pr.robot_maxAx;
    model.acc_y_min     =   -pr.robot_maxAy;
    model.acc_y_max     =    pr.robot_maxAy;
    
    %% Dynamics
    expr_f_expl = vertcat(ego_state(3), ...     % x1 dot
                          ego_state(4), ...     % x2 dot
                          ego_inputs(1), ...    % x3 dot
                          ego_inputs(2));       % x4 dot
    model.x0 = zeros(model.nx, 1);              % just give dimension, changed in real time
    
    %% Collision avoidance nonlinear constraints
    a = p_rob_size(1) + 1.05*p_obs_size(1);
    b = p_rob_size(2) + 1.05*p_obs_size(2);
    d = ego_state(1:2) - p_obs_pos;
    ego_obs_dis = d(1)^2/a^2 + d(2)^2/b^2 -1;   % ego_obs_dis > 0
    
    %% Cost
    % waypoint navigation cost
    cost_wp_pos = w_pos * obj_desired_pos(ego_state(1:2), p_rob_start, p_rob_goal);
    % control input cost
    cost_input_acc = w_inputs * obj_input_acc(ego_inputs, pr);
    % collision potentials cost
    cost_coll = w_coll * obj_collision_potential(ego_state(1:2), p_rob_size, ...
        p_obs_pos, p_obs_size, p_obs_scale);
    % stage cost
    cost = cost_wp_pos + cost_input_acc + cost_coll;
    % terminal cost
    cost_e = cost_wp_pos + cost_coll;
       
    %% Populate struct
    model.x = ego_state;                    % state
    model.u = ego_inputs;                   % control
    model.xdot = ego_state_dot;             % state derivative
    model.p = para;                         % parameter
    model.dyn.expr_f = expr_f_expl;         % dynamics, f
    model.constr.expr_h = ego_obs_dis;      % stage inequality constraint
    model.constr.expr_h_e = ego_obs_dis;    % terminal inequality constraint
    model.constr.dis_min = 0;               % min safe distance
    model.constr.dis_max = Inf;             % max safe distance
    model.cost.expr_ext_cost = cost;        % stage cost
    model.cost.expr_ext_cost_e = cost_e;    % terminal cost
    model.w_slack = w_slack;                % slack cost weight (passing to ocp model next)

end
