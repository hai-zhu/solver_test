%% formulate the NLP using casadi syntax
function [nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg] = ...
    casadi_shooting_nlp_form(model)

    import casadi.*         % to use CasADi sytax

    global index pr
    
    %% Define stage variables
    % state
    ego_state = MX.sym('ego_state', model.nx);
    ego_pos = ego_state(index.x.pos);
    % control
    ego_inputs = MX.sym('ego_inputs', model.nu);
    % slack
    slack = MX.sym('slack', model.ns);
    % parameters
    p_stage = MX.sym('p_stage', model.np);
    p_rob_start = p_stage(index.p.robot_start);
    p_rob_goal  = p_stage(index.p.robot_goal);
    p_rob_size  = p_stage(index.p.robot_size);
    p_mpc_weights = p_stage(index.p.mpc_weights);
    p_obs_pos   = p_stage(index.p.obs_pos);
    p_obs_size  = p_stage(index.p.obs_size);
    p_obs_scale = p_stage(index.p.obs_scale);
    % mpc weights
    w_pos       =   p_mpc_weights(1);
    w_inputs    =   p_mpc_weights(2);
    w_coll      =   p_mpc_weights(3);
    w_slack     =   p_mpc_weights(4);
    
    %% Define dynamics function f
    X0 = MX.sym('X0', model.nx);
    U = MX.sym('U', model.nu);
    Xf = my_RK2(X0, U, @pr.robot_dynamics_continuous, model.dt, []);
    F = Function('F', {X0, U}, {Xf}, {'x0', 'u'}, {'xf'});
    
    %% Define objective function J
    % control input cost
    cost_input_acc = w_inputs * obj_input_acc(ego_inputs, pr);
    obj_input = Function('obj_input', {ego_inputs, p_stage}, {cost_input_acc});
    % waypoint navigation cost
    cost_wp_pos = w_pos * obj_desired_pos(ego_pos, p_rob_start, p_rob_goal);
    obj_wp = Function('obj_wp', {ego_state, p_stage}, {cost_wp_pos});
    % collision potentials cost
    cost_coll = w_coll * obj_collision_potential(ego_state(1:2), p_rob_size, ...
        p_obs_pos, p_obs_size, p_obs_scale);
    obj_coll = Function('obj_coll', {ego_state, p_stage}, {cost_coll});
    % slack cost
    cost_slack = w_slack * slack^2;
    obj_slack = Function('obj_slack', {slack, p_stage}, {cost_slack});
    
    %% Define inequality constraints h
    a = p_rob_size(1) + 1.05*p_obs_size(1);
    b = p_rob_size(2) + 1.05*p_obs_size(2);
    d = ego_state(1:2) - p_obs_pos;
    ego_obs_dis = d(1)^2/a^2 + d(2)^2/b^2 -1 + slack;   % ego_obs_dis > 0
    h_ego_obs_dis = Function('h_ego_obs_dis', {ego_state, p_stage, slack}, {ego_obs_dis});    

    %% Start with an empty NLP
    nlp_x   =   {};     % decision variables
    nlp_x0  =   [];     % initial guess on w
    nlp_lbx =   [];     % lower bound on w
    nlp_ubx =   [];     % upper bound on w
    nlp_J   =   0;      % cost accumulator
    nlp_g   =   {};     % inequality constraint equations
    nlp_lbg =   [];     % lower bound on g
    nlp_ubg =   [];     % upper bound on g
    nlp_p   =   {};     % real time parameter
    
    %% "Lift" initial conditions
    Xk      = MX.sym('X0', model.nx);       % initial state of each control interval
    nlp_x   = [nlp_x; {Xk}];                % initial state as decision variable
    nlp_lbx = [nlp_lbx; zeros(model.nx, 1)];% adding bound to enforce initial condition
    nlp_ubx = [nlp_ubx; zeros(model.nx, 1)];
    nlp_x0  = [nlp_x0; zeros(model.nx, 1)];	% adding initial guess
    
    %% Formulate the NLP
    for k = 0 : model.N-1
        % Parameters
        Pk = MX.sym(['P_' num2str(k)], model.np);   % parameter of this stage
        nlp_p = [nlp_p; {Pk}];

        % New NLP variable for the control
        Uk = MX.sym(['U_' num2str(k)], model.nu);
        nlp_x = [nlp_x; {Uk}];          % adding u as decision variable
        nlp_lbx = [nlp_lbx; model.ul]; 	% adding bound for the decision variable
        nlp_ubx = [nlp_ubx; model.uu];
        nlp_x0 = [nlp_x0; zeros(model.nu, 1)];  % adding initial guess

        % Integrate till the end of the interval
        Fk = F('x0', Xk, 'u', Uk);
        Xk_end = Fk.xf;
        
        % New NLP variable for state at end of interval
        Xk = MX.sym(['X_' num2str(k+1)], model.nx);
        nlp_x = [nlp_x; {Xk}];          % adding xk as decision variable
        nlp_lbx = [nlp_lbx; model.xl];
        nlp_ubx = [nlp_ubx; model.xu];
        nlp_x0 = [nlp_x0; zeros(model.nx, 1)];
        
        % Slack
        Sk = MX.sym(['S_' num2str(k+1)], model.ns);
        nlp_x = [nlp_x; {Sk}];          % adding sk as decision variable
        nlp_lbx = [nlp_lbx; 0*ones(model.ns, 1)];
        nlp_ubx = [nlp_ubx; Inf*ones(model.ns, 1)];
        nlp_x0 = [nlp_x0; zeros(model.ns, 1)];

        % Add equality constraint
        nlp_g = [nlp_g; {Xk_end - Xk}];
        nlp_lbg = [nlp_lbg; zeros(model.nx, 1)];
        nlp_ubg = [nlp_ubg; zeros(model.nx, 1)];
        
        % Add path constraint
        nlp_g = [nlp_g; {h_ego_obs_dis(Xk, Pk, Sk)}];
        nlp_lbg = [nlp_lbg; 0];
        nlp_ubg = [nlp_ubg; Inf];
        
        % cumulate stage cost
        nlp_J = nlp_J + obj_input(Uk, Pk);
        nlp_J = nlp_J + obj_wp(Xk, Pk);
        nlp_J = nlp_J + obj_coll(Xk, Pk);
        nlp_J = nlp_J + obj_slack(Sk, Pk);
        
    end
    
    %% Create an NLP solver
    % problem definition
    prob = struct();
    prob.x = vertcat(nlp_x{:});     % decision variable
    prob.f = nlp_J;                 % objective to be minimized
    prob.g = vertcat(nlp_g{:});     % inequality constraints
    prob.p = vertcat(nlp_p{:});     % parameter
    % solver options
    opts = struct();                % solver options
    opts.print_time = 0;
    opts.ipopt.print_level = 0;
    opts.ipopt.print_user_options = 'no';
    opts.ipopt.print_options_documentation = 'no';
    opts.ipopt.print_timing_statistics = 'no';
    opts.ipopt.max_iter = 100;
    opts.ipopt.tol = 1E-4;
%     opts.ipopt.linear_solver = 'ma77';
    % create the solver
    nlp_solver = nlpsol('solver', 'ipopt', prob, opts);

end
