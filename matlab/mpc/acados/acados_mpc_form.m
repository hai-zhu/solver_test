function ocp_model = acados_mpc_form(model)

    % formulate acados ocp model
    ocp_model = acados_ocp_model();         % construction
    
    ocp_model.set('name', model.name);      % name
    ocp_model.set('T', model.T);            % horizon length, s
    
    %% symbolics
    ocp_model.set('sym_x', model.x);        % state
    ocp_model.set('sym_u', model.u);        % control
    ocp_model.set('sym_xdot', model.xdot);  % state derivative
    ocp_model.set('sym_p', model.p);        % parameter
    
    %% dynamics
    ocp_model.set('dyn_type', 'explicit');  % dynamics type
    ocp_model.set('dyn_expr_f', model.dyn.expr_f);  % dynamics model, f
    
    %% set intial condition
    ocp_model.set('constr_x0', model.x0);   % initial constraint
    
    %% path constraints, linear (bounds)
    % state bound
    Jbx = eye(model.nx);
    ocp_model.set('constr_Jbx', Jbx);
    ocp_model.set('constr_lbx', [model.pos_x_min, model.pos_y_min, model.vel_x_min, model.vel_y_min]);
    ocp_model.set('constr_ubx', [model.pos_x_max, model.pos_y_max, model.vel_x_max, model.vel_y_max]);
    % control bound
    Jbu = eye(model.nu);
    ocp_model.set('constr_Jbu', Jbu);
    ocp_model.set('constr_lbu', [model.acc_x_min, model.acc_y_min]);
    ocp_model.set('constr_ubu', [model.acc_x_max, model.acc_y_max]);
    
    %% path constraints, nonlinear collision avoidance
    % stage
    ocp_model.set('constr_expr_h', model.constr.expr_h);
    ocp_model.set('constr_lh', model.constr.dis_min);
    ocp_model.set('constr_uh', model.constr.dis_max);
    % terminal
    ocp_model.set('constr_expr_h_e', model.constr.expr_h_e);
    ocp_model.set('constr_lh_e', model.constr.dis_min);
    ocp_model.set('constr_uh_e', model.constr.dis_max);
    
    %% config constraints slack variables, comment if not using slack
    %% stage
    nsh = 1;
    Jsh = zeros(model.nh, nsh);
    Jsh(1,1) = 1;
    ocp_model.set('constr_Jsh', Jsh);
    % L1 slack (linear term)
    ocp_model.set('cost_zl', 0 * ones(nsh,1));
    ocp_model.set('cost_zu', 0 * ones(nsh,1));
    % L2 slack (squared term)
    ocp_model.set('cost_Zl', 1E4 * eye(nsh,nsh));
    ocp_model.set('cost_Zu', 0 * eye(nsh,nsh));
    %% terminal
    nsh_e = 1;
    Jsh_e = zeros(model.nh, nsh_e);
    Jsh_e(1,1) = 1;
    ocp_model.set('constr_Jsh_e', Jsh_e);
    % L1 slack (linear term)
    ocp_model.set('cost_zl_e', 0 * ones(nsh_e,1));
    ocp_model.set('cost_zu_e', 0 * ones(nsh_e,1));
    % L2 slack (squared term)
    ocp_model.set('cost_Zl_e', 1E4 * eye(nsh_e,nsh_e));
    ocp_model.set('cost_Zu_e', 0 * eye(nsh_e,nsh_e));

    %% costs
    % stage  
    ocp_model.set('cost_type', 'ext_cost');
    ocp_model.set('cost_expr_ext_cost', model.cost.expr_ext_cost);
    % terminal
    ocp_model.set('cost_type_e', 'ext_cost');
    ocp_model.set('cost_expr_ext_cost_e', model.cost.expr_ext_cost_e);
    
end