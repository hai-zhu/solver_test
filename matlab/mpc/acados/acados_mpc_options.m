function ocp_opts = acados_mpc_options(model)

    %% acados ocp set opts
    ocp_opts = acados_ocp_opts();
    
    % integrator
    ocp_opts.set('sim_method', 'erk');          % explicit RK
    ocp_opts.set('sim_method_num_stages', 2);   % 1-RK1, 2-RK2, 4-RK4
    ocp_opts.set('sim_method_num_steps', 1);
    ocp_opts.set('sim_method_newton_iter', 3);
    ocp_opts.set('gnsf_detect_struct', 'true');
    
    % shooting methods
    ocp_opts.set('param_scheme_N', model.N);
    
    % code generation
    ocp_opts.set('compile_interface', 'auto');
    if model.new_solver
        ocp_opts.set('codgen_model', 'true');
        ocp_opts.set('compile_model', 'true');
    else
        ocp_opts.set('codgen_model', 'false');
        ocp_opts.set('compile_model', 'false');
    end
    ocp_opts.set('output_dir', model.output_dir);
    
    % NLP solver
    ocp_opts.set('nlp_solver', 'sqp');
    ocp_opts.set('nlp_solver_max_iter', 100);
    ocp_opts.set('nlp_solver_tol_stat', 1E-4);
    ocp_opts.set('nlp_solver_tol_eq', 1E-4);
    ocp_opts.set('nlp_solver_tol_ineq', 1E-4);
    ocp_opts.set('nlp_solver_tol_comp', 1E-4);
    ocp_opts.set('nlp_solver_step_length', 0.2);
    ocp_opts.set('nlp_solver_ext_qp_res', 1);

    % QP solver
    ocp_opts.set('qp_solver', 'full_condensing_qpoases');   % should try and compare different solvers
    ocp_opts.set('qp_solver_cond_N', 5); % floor(model.N/2)
    ocp_opts.set('qp_solver_iter_max', 60);
    ocp_opts.set('qp_solver_warm_start', 0);
    ocp_opts.set('qp_solver_cond_ric_alg', 1);
    ocp_opts.set('qp_solver_ric_alg', 1);
    
    % globalization
    
    % Hessian approximation
    ocp_opts.set('nlp_solver_exact_hessian', 'false');
    ocp_opts.set('regularize_method', 'no_regularize');
    
    % other
    ocp_opts.set('print_level', 0);
    
   
end
