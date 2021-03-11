%% formulate the NLP using casadi syntax
function [nlp_solver, nlp_lbx, nlp_ubx, nlp_lbg, nlp_ubg] = ...
    casadi_collocation_nlp_form(model)

    import casadi.*         % to use CasADi sytax

    global index pr
    
    %% Define stage variables
    px = MX.sym('px');
    py = MX.sym('py');
    vx = MX.sym('vx');
    vy = MX.sym('vy');
    x_stage = vertcat(px, py, vx, vy);
    % control
    ax = MX.sym('ax');
    ay = MX.sym('ay');
    u_stage = vertcat(ax, ay);
    % slack
    s_ca = MX.sym('s_ca');
    s_stage = s_ca;
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
    
    %% Define model equations
    px_dot = vx;
    py_dot = vy;
    vx_dot = ax;
    vy_dot = ay;
    x_stage_dot = vertcat(px_dot, py_dot, vx_dot, vy_dot);
    dyn_f = Function('dyn_f', {x_stage, u_stage}, {x_stage_dot});
        
    %% Define objective function J
    % control input cost
    cost_input_acc = w_inputs * obj_input_acc(u_stage, pr);
    obj_input = Function('obj_input', {u_stage, p_stage}, {cost_input_acc});
    % waypoint navigation cost
    cost_wp_pos = w_pos * obj_desired_pos(x_stage(1:2), p_rob_start, p_rob_goal);
    obj_wp = Function('obj_wp', {x_stage, p_stage}, {cost_wp_pos});
    % collision potentials cost
    cost_coll = w_coll * obj_collision_potential(x_stage(1:2), p_rob_size, ...
        p_obs_pos, p_obs_size, p_obs_scale);
    obj_coll = Function('obj_coll', {x_stage, p_stage}, {cost_coll});
    % slack cost
    cost_slack = w_slack * s_stage^2;
    obj_slack = Function('obj_slack', {s_stage, p_stage}, {cost_slack});
    
    %% Define inequality constraints h
    a = p_rob_size(1) + 1.05*p_obs_size(1);
    b = p_rob_size(2) + 1.05*p_obs_size(2);
    d = x_stage(1:2) - p_obs_pos;
    ego_obs_dis = d(1)^2/a^2 + d(2)^2/b^2 -1 + s_stage;   % ego_obs_dis > 0
    h_ego_obs_dis = Function('h_ego_obs_dis', {x_stage, p_stage, s_stage}, {ego_obs_dis}); 
    
    %% Definitions fo collocation
    DT = model.dt;
    XK = MX.sym('XK', model.nx);
    XK1 = MX.sym('XK1', model.nx);
    XDOTK = MX.sym('XDOTK', model.nx);
    XDOTK1 = MX.sym('XDOTK1', model.nx);
    UK = MX.sym('UK', model.nu);
    UK1 = MX.sym('UK1', model.nu);
    X_tc = 1/2 * (XK + XK1) + DT/8 * (XDOTK - XDOTK1);
    f_x_tc = Function('f_x_tc', {XK, XK1, XDOTK, XDOTK1}, {X_tc}, {'xk', 'xk1', 'xdk', 'xdk1'}, {'xtc'});
    Xdot_tc = -3/(2*DT)*(XK - XK1) - 1/4 * (XDOTK + XDOTK1);
    f_xdot_tc = Function('f_xdot_tc', {XK, XK1, XDOTK, XDOTK1}, {Xdot_tc}, {'xk', 'xk1', 'xdk', 'xdk1'}, {'xdtc'});
    U_tc = 1/2 * (UK + UK1);
    f_u_tc = Function('f_u_tc', {UK, UK1}, {U_tc}, {'uk', 'uk1'}, {'utc'});

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
    
    Uk      = MX.sym('U0', model.nu);
    nlp_x = [nlp_x; {Uk}];
    nlp_lbx = [nlp_lbx; model.ul];
    nlp_ubx = [nlp_ubx; model.uu];
    nlp_x0 = [nlp_x0; zeros(model.nu, 1)];
    
    %% Formulate the NLP
    for k = 0 : model.N-1
        
        % Parameters
        Pk = MX.sym(['P_' num2str(k)], model.np);   % parameter of this stage
        nlp_p = [nlp_p; {Pk}];
        
        % New NLP variables for the state
        Xk_next = MX.sym(['X_' num2str(k+1)], model.nx);
        nlp_x   = [nlp_x; {Xk_next}];                
        nlp_lbx = [nlp_lbx; model.xl];
        nlp_ubx = [nlp_ubx; model.xu];
        nlp_x0  = [nlp_x0; zeros(model.nx, 1)];
        
        % New NLP variables for the control
        Uk_next = MX.sym(['U_' num2str(k+1)], model.nu);
        nlp_x   = [nlp_x; {Uk_next}];
        nlp_lbx = [nlp_lbx; model.ul];
        nlp_ubx = [nlp_ubx; model.uu];
        nlp_x0  = [nlp_x0; zeros(model.nu, 1)];
        
        % Collocation constraints
        Xdot_k  = dyn_f(Xk, Uk);
        Xdot_k1 = dyn_f(Xk_next, Uk_next);
        x_col_eval = f_x_tc('xk', Xk, 'xk1', Xk_next, ...
            'xdk', Xdot_k, 'xdk1', Xdot_k1);
        x_col = x_col_eval.xtc; 
        u_col_eval = f_u_tc('uk', Uk, 'uk1', Uk_next);
        u_col = u_col_eval.utc;
        xdot_col_eval = f_xdot_tc('xk', Xk, 'xk1', Xk_next, ...
            'xdk', Xdot_k, 'xdk1', Xdot_k1);
        xdot_col = xdot_col_eval.xdtc;
        % collocation state constraints
        nlp_g = [nlp_g; {xdot_col - dyn_f(x_col, u_col)}];
        nlp_lbg = [nlp_lbg; zeros(model.nx, 1)];
        nlp_ubg = [nlp_ubg; zeros(model.nx, 1)];
        
        % Slack
        Sk = MX.sym(['S_' num2str(k+1)], model.ns);
        nlp_x = [nlp_x; {Sk}];          % adding sk as decision variable
        nlp_lbx = [nlp_lbx; 0*ones(model.ns, 1)];
        nlp_ubx = [nlp_ubx; Inf*ones(model.ns, 1)];
        nlp_x0 = [nlp_x0; zeros(model.ns, 1)];
        
        % Add path constraint
        nlp_g = [nlp_g; {h_ego_obs_dis(Xk, Pk, Sk)}];
        nlp_lbg = [nlp_lbg; 0];
        nlp_ubg = [nlp_ubg; Inf];
        
        % cumulate stage cost
        nlp_J = nlp_J + obj_input(Uk, Pk);
        nlp_J = nlp_J + obj_wp(Xk, Pk);
        nlp_J = nlp_J + obj_coll(Xk, Pk);
        nlp_J = nlp_J + obj_slack(Sk, Pk);
        
        % prepare for next loop
        Xk = Xk_next;
        Uk = Uk_next;
        
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
