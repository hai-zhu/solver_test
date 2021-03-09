function ineq = forces_pro_ineq(z, p)

    % define nonlinear inequalities for mpc

    global index                            % global index information
    
    %% information from state
    ego_pos     =   z(index.z.pos);
    ego_slack   =   z(index.z.slack);
    
    %% information from online parameter
    ego_size    =   p(index.p.robot_size);
    obs_pos     =   p(index.p.obs_pos);
    obs_size    =   p(index.p.obs_size);
    
    %% collision avoidance constraints
    a = ego_size(1) + 1.05*obs_size(1);
    b = ego_size(2) + 1.05*obs_size(2);
    d = ego_pos - obs_pos;
    cons_obs = d(1)^2/a^2 + d(2)^2/b^2 -1 + ego_slack;
    
    %% combine inequality constraints
    ineq = cons_obs;

end
