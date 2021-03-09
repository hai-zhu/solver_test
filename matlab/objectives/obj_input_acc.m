function cost = obj_input_acc(ego_input_acc, pr)
    
    % Compute the control input cost, normalization is used
    
    ego_input_normalized = [ego_input_acc(1)/pr.robot_maxAx; ...
        ego_input_acc(2)/pr.robot_maxAy];
    Q = diag([1.0; 1.0]);	% can be ajusted to penalize more along some direction
    cost = ego_input_normalized' * Q * ego_input_normalized;
    
end
