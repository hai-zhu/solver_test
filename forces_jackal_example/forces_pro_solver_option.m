function codeoptions = forces_pro_solver_option(solver_name)

    % setting solver generation options of Forces Pro
    codeoptions = getOptions(solver_name);	% default solver options
    
    %% general options
    codeoptions.printlevel  = 0;        % 0-none; 1-each solve; 2-each iteration
    codeoptions.maxit       = 200;      % maximum number of iterations
    codeoptions.optlevel    = 0;        % 0: no optimization, 
                                        % 1: optimize for size, 
                                        % 2: optimize for speed, 
                                        % 3: optimize for size & speed
                                        % use 0 for test, 3 for depolyment
    codeoptions.threadSafeStorage = true; % the generated solver can be run
                                          % in parallel on different threads                                    
    codeoptions.timing      = 1;        % measure the time used for executing the generated code
    
%     codeoptions.solver_timeout = 0;     % 0: not checking the execution time of each iteration
                                        % 1: solver_timeout (default -1.0)
                                        % 2: solver_timeout, timeout_estimate_coeff (default 1.2)
%     codeoptions.sqp_nlp.qp_timeout = 0; % used when enable solver_timeout
    codeoptions.overwrite   = 1;        % overwrite existing solver
    codeoptions.showinfo    = 0;        % solver info in simulink block
%     codeoptions.server      = 'https://www.embotech.com/codegen';
    codeoptions.BuildSimulinkBlock = 0; % skipping builing of simulink S-function
    codeoptions.cleanup     = 1;        % automatically clean up some files during generation
%     codeoptions.platform    = 'Generic';% target platform
    
    %% high-level interface options
%     codeoptions.nlp.integrator.type = 'ERK2'; % integrator
%     codeoptions.nlp.integrator.Ts = 0.01;
%     codeoptions.nlp.integrator.nodes = 10;
    codeoptions.nlp.TolStat = 1E-5;     % infinity norm tolerance on stationarity
    codeoptions.nlp.TolEq   = 1E-6;     % tol. on equality constraints
    codeoptions.nlp.TolIneq = 1E-6;     % tol. on inequality constraints
    codeoptions.nlp.TolComp = 1E-3;     % tol. on complementarity
%     codeoptions.nlp.BarrStrat = 'loqo'; % barrier parameter
%     codeoptions.nlp.hessian_approximation = 'bfgs'; % Hessian approximation method 
%     codeoptions.nlp.linear_solver = 'symm_indefinite_fast'; % linear system solver
%     codeoptions.nlp.ad_tool = 'casadi-3.5.1'; % automatic differentiation (AD) tool , or casadi-3.5.1
    codeoptions.noVariableElimination = 1;
    
    %% solve method
%     codeoptions.parallel    = 1;        % enable computation on multiple cores

end
