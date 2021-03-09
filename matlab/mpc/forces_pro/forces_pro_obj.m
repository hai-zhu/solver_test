function cost = forces_pro_obj(z, p)

    global index pr
    
    %% information from state
    ego_pos     =   z(index.z.pos);
    ego_inputs  =   z(index.z.inputs);
    ego_slack   =   z(index.z.slack);
    
    %% information from online parameter
    ego_start   =   p(index.p.robot_start);
    ego_goal    =   p(index.p.robot_goal);
    ego_size    =   p(index.p.robot_size);
    w_pos       =   p(index.p.mpc_weights(1));
    w_inputs    =   p(index.p.mpc_weights(2));
    w_coll      =   p(index.p.mpc_weights(3));
    w_slack     =   p(index.p.mpc_weights(4));
    obs_pos     =   p(index.p.obs_pos);
    obs_size    =   p(index.p.obs_size);
    obs_scale   =   p(index.p.obs_scale);
       
    %% waypoint cost
    cost_wp_pos = w_pos * obj_desired_pos(ego_pos, ego_start, ego_goal);

    %% control input cost
    cost_input_acc = w_inputs * obj_input_acc(ego_inputs, pr);
    
    %% collision potential cost
    cost_coll = w_coll * obj_collision_potential(ego_pos, ego_size, ...
        obs_pos, obs_size, obs_scale);
    
    %% slack cost
    cost_slack = w_slack * ego_slack;
    
    %% combine all cost
    cost = cost_wp_pos + cost_input_acc + cost_coll + cost_slack;
    
end
