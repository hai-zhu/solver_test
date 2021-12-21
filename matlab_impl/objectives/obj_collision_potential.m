function cost = obj_collision_potential(ego_pos, ego_size, obs_pos, obs_size, obs_scale)

    % Compute the potential filed based collision avoidance cost
    
    a = ego_size(1) + obs_scale*obs_size(1);
    b = ego_size(2) + obs_scale*obs_size(2);
    
    d_vec = ego_pos - obs_pos;
    d = d_vec(1)^2/a^2 + d_vec(2)^2/b^2;
    % if else based
%     d_c = d - 1;
%     cost = if_else(d_c>0, 0, -d_c);
    % logistic based
    cost = 1 / (1+exp(10*(d - obs_scale)));

end
