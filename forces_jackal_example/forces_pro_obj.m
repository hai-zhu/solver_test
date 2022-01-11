function cost = forces_pro_obj(z, p)

    global pr index
    
    %% information from state
    ego_inputs  =   z(index.z_inputs);
    ego_states  =   z(index.z_states);
    ego_pos     =   ego_states(index.x_pos);
    
    %% information from online parameter
    ego_start   =   p(index.p_robot_pos_start);
    ego_goal    =   p(index.p_robot_pos_goal);
    ego_size    =   p(index.p_robot_size);
    obs_pos     =   p(index.p_obs_pos);
    obs_size    =   p(index.p_obs_size);
    mpc_weights =   p(index.p_mpc_weights);
    w_pos       =   mpc_weights(index.p_mpc_weights_w_pos);
    w_inputs    =   mpc_weights(index.p_mpc_weights_w_input);
    w_coll      =   mpc_weights(index.p_mpc_weights_w_coll);
     
    %% waypoint cost
    cost_wp_pos = w_pos * obj_desired_pos(ego_pos, ego_start, ego_goal);

    %% control input cost
    ego_inputs_normalized = [ego_inputs(1)/pr.robot_maxAcc; ego_inputs(2)/pr.robot_maxOmegaAcc];
    cost_input = w_inputs * obj_input_acc(ego_inputs_normalized);
    
    %% collision potential cost
    cost_coll = w_coll * obj_collision_potential(ego_pos, ego_size, ...
        obs_pos, obs_size);
    
    %% combine all cost
    cost = cost_wp_pos + cost_input + cost_coll;
    
end


function cost = obj_input_acc(ego_input)
    
    % Compute the control input cost, normalization is used
    Q = diag([1.0; 1.0]);	% can be ajusted to penalize more along some direction
    cost = ego_input' * Q * ego_input;
    
end



function cost = obj_desired_pos(ego_pos, ego_start_pos, ego_goal_pos)
    
    % Compute the goal position progressing cost, normalization is used
    lenToGoal   =   (ego_goal_pos - ego_start_pos)' ...
        * (ego_goal_pos - ego_start_pos);   % length between current start
                                            % and goal position, using
                                            % quadratic form
    lenToGoal   =   max(lenToGoal, 1);      % in case arriving at goal posistion
    Q = diag([1.0; 1.0]);                   % x, y the same weight
    cost = (ego_goal_pos - ego_pos)' * Q ...
        * (ego_goal_pos - ego_pos) / lenToGoal;
    
end



function cost = obj_collision_potential(ego_pos, ego_size, obs_pos, obs_size)

    % Compute the potential filed based collision avoidance cost
    a = ego_size(1) + 1.6*obs_size(1);
    b = ego_size(2) + 1.6*obs_size(2);
    d_vec = ego_pos - obs_pos;
    d = d_vec(1)^2/a^2 + d_vec(2)^2/b^2;
    % if else based
    d_c = d - 1;
    cost = if_else(d_c>0, 0, -d_c);
    % logistic based
%     cost = 1 / (1+exp(10*(d - 1.6)));

end

