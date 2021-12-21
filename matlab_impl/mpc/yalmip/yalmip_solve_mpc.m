function [output, exitflag, info] = yalmip_solve_mpc(model, problem, options)

    % formulate and call a sovler (Ipopt) to solve the MPC problem
    global index pr
    
    %% define decision variables
    x = sdpvar(model.neq, model.N+1);       % state, x(1:N+1)
    u = sdpvar(model.nin, model.N);         % control inputs, u(1:N)
    s = sdpvar(model.nslack, model.N);      % slack, s(1:N)
    
    %% fetch parameters
    robot_start_all_stage = problem.parameters_all_stage(index.p.robot_start, :);
    robot_goal_all_stage  = problem.parameters_all_stage(index.p.robot_goal, :);
    robot_size_all_stage  = problem.parameters_all_stage(index.p.robot_size, :);
    mpc_weights_all_stage = problem.parameters_all_stage(index.p.mpc_weights, :);
    obs_pos_all_stage     = problem.parameters_all_stage(index.p.obs_pos, :);
    obs_size_all_stage    = problem.parameters_all_stage(index.p.obs_size, :);
    obs_scale_all_stage   = problem.parameters_all_stage(index.p.obs_scale, :);
    
    %% formulate constraints and costs
    dynamics_constraints = [];
    state_space_constraints = [];
    input_space_constraints = [];
    slack_space_constraints = [];
    coll_avoid_constraints = [];
    cost = 0;
    % initial conditions
    dynamics_constraints = [dynamics_constraints; x(:,1) == problem.xinit];
    for iStage = 1 : model.N
        % parameters used
        ego_start = robot_start_all_stage(:, iStage);
        ego_goal  = robot_goal_all_stage(:, iStage);
        ego_size  = robot_size_all_stage(:, iStage);
        mpc_weights = mpc_weights_all_stage(:, iStage);
        obs_pos   = obs_pos_all_stage(:, iStage);
        obs_size  = obs_size_all_stage(:, iStage);
        obs_scale = obs_scale_all_stage(:, iStage);
        ego_x_previous = x(:, iStage);              % state before u
        ego_x = x(:, iStage+1);                     % state after u
        ego_u = u(:, iStage);
        ego_slack = s(:, iStage);
        ego_pos = ego_x(index.x.pos);
        % dynamics constraint
        dynamics_constraints = [dynamics_constraints; ...
            ego_x == my_RK2(ego_x_previous, ego_u, pr.robot_dynamics_continuous, model.dt, [])];
        % state space constraint
        state_space_constraints = [state_space_constraints; ...
            model.xl <= ego_x <= model.xu];
        % input space constraint
        input_space_constraints = [input_space_constraints; ...
            model.ul <= ego_u <= model.uu];
        % slack space constraint
        slack_space_constraints = [slack_space_constraints; ...
            model.sl <= ego_slack <= model.su];
        % collision avoidance constraint
        a = ego_size(1) + 1.05*obs_size(1);
        b = ego_size(2) + 1.05*obs_size(2);
        d = ego_pos - obs_pos;
        cons_obs = d(1)^2/a^2 + d(2)^2/b^2 -1 + ego_slack;
        coll_avoid_constraints = [coll_avoid_constraints; cons_obs >= 0];
        % cost
        cost_wp_pos = mpc_weights(1) * obj_desired_pos(ego_pos, ego_start, ego_goal);
        cost_input_acc = mpc_weights(2) * obj_input_acc(ego_u, pr);
        cost_coll = mpc_weights(3) * obj_collision_potential(ego_pos, ego_size, ...
            obs_pos, obs_size, obs_scale);
        cost_slack = mpc_weights(4) * ego_slack;
        cost = cost + cost_wp_pos + cost_input_acc + cost_coll + cost_slack;
    end
    % all constraints
    constraints = [dynamics_constraints; state_space_constraints; ...
        input_space_constraints; slack_space_constraints; coll_avoid_constraints];
    
    %% calling the solver
    diagnostics = optimize(constraints, cost, options);
    if diagnostics.problem == 0  % feasible
        exitflag = 1;
        u_plan = value(u);      % 1:N
        s_plan = value(s);      % 1:N
        x_plan = value(x);      % 1:N+1
        output.z_plan = [u_plan; s_plan; x_plan(:,2:end)];
    else
        exitflag = 0;
        warning('MPC infeasible!')
        output.z_plan = [];
    end
    info.yalmiptime = diagnostics.yalmiptime;
    info.solvetime = diagnostics.solvertime;    

end
