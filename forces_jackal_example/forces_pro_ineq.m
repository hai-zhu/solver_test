function ineq = forces_pro_ineq(z, p)

    % define nonlinear inequalities for mpc

    global index                            % global index information
    
    %% information from state
    ego_states  =   z(index.z_states);
    ego_pos     =   ego_states(index.x_pos);
    ego_slacks  =   z(index.z_slacks);
    slack_coll  =   ego_slacks(index.s_coll);
    
    %% information from online parameter
    ego_size    =   p(index.p_robot_size);
    obs_pos     =   p(index.p_obs_pos);
    obs_size    =   p(index.p_obs_size);
    
    %% collision avoidance constraints
    a = ego_size(1) + 1.05*obs_size(1);
    b = ego_size(2) + 1.05*obs_size(2);
    d = ego_pos - obs_pos;
    cons_obs = d(1)^2/a^2 + d(2)^2/b^2 -1 + slack_coll;
    
    %% combine inequality constraints
    ineq = cons_obs;

end
