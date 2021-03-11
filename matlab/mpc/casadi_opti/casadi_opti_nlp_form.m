%% formulate the NLP using casadi Opti stack
function opti = casadi_opti_nlp_form(model)

    global index pr

    opti = casadi.Opti();

    %% define decision variables and parameters
    u = opti.variable(model.nin, model.N);      % control inputs, u(1:N)
    s = opti.variable(model.nslack, model.N);   % slack, s(1:N)
    x = opti.variable(model.neq, model.N+1);    % state, x(1:N+1)
    p = opti.parameter(model.npar, model.N);    % parameter

    %% parameter details
    robot_state_all_stage = p(index.p.robot_state, :);
    robot_start_all_stage = p(index.p.robot_start, :);
    robot_goal_all_stage  = p(index.p.robot_goal, :);
    robot_size_all_stage  = p(index.p.robot_size, :);
    mpc_weights_all_stage = p(index.p.mpc_weights, :);
    obs_pos_all_stage     = p(index.p.obs_pos, :);
    obs_size_all_stage    = p(index.p.obs_size, :);
    obs_scale_all_stage   = p(index.p.obs_scale, :);

    %% formulate constraints and costs
    cost = 0;
    % initial conditions
    opti.subject_to(x(:,1) == robot_state_all_stage(:, 1));
    for iStage = 1 : model.N
        % parameters used
        ego_start = robot_start_all_stage(:, iStage);
        ego_goal  = robot_goal_all_stage(:, iStage);
        ego_size  = robot_size_all_stage(:, iStage);
        mpc_weights = mpc_weights_all_stage(:, iStage);
        obs_pos   = obs_pos_all_stage(:, iStage);
        obs_size  = obs_size_all_stage(:, iStage);
        obs_scale = obs_scale_all_stage(:, iStage);
        ego_x_previous = x(:, iStage);              % state at u
        ego_x = x(:, iStage+1);                     % state after u
        ego_u = u(:, iStage);                       % current u
        ego_slack = s(:, iStage);                   % current slack
        ego_pos = ego_x(index.x.pos);
        % dynamics constraint
        opti.subject_to(ego_x == my_RK2(ego_x_previous, ego_u, model.robot_dynamics_continuous, model.dt, []));
        % state space constraint
        opti.subject_to(model.xl <= ego_x);
        opti.subject_to(ego_x <= model.xu);
        % input space constraint
        opti.subject_to(model.ul <= ego_u);
        opti.subject_to(ego_u <= model.uu);
        % slack space constraint
        opti.subject_to(model.sl <= ego_slack);      
        opti.subject_to(ego_slack <= model.su);
        % collision avoidance constraint
        a = ego_size(1) + 1.05*obs_size(1);
        b = ego_size(2) + 1.05*obs_size(2);
        d = ego_pos - obs_pos;
        cons_obs = d(1)^2/a^2 + d(2)^2/b^2 -1 + ego_slack;
        opti.subject_to(cons_obs >= 0);
        % cost
        cost_wp_pos = mpc_weights(1) * obj_desired_pos(ego_pos, ego_start, ego_goal);
        cost_input_acc = mpc_weights(2) * obj_input_acc(ego_u, pr);
        cost_coll = mpc_weights(3) * obj_collision_potential(ego_pos, ego_size, ...
            obs_pos, obs_size, obs_scale);
        cost_slack = mpc_weights(4) * ego_slack^2;
        cost = cost + cost_wp_pos + cost_input_acc + cost_coll + cost_slack;
    end

    %% solver options
    opti.minimize(cost);
    options.print_time = 0;
    options.ipopt.print_level = 0;
    options.ipopt.tol = 1E-6;
    options.ipopt.max_iter = 300;
    opti.solver('ipopt',options);

end