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
