function msg = forces_pro_solver_exitflag(exitflag)

    % printing exitflag info of the forces pro solver

    switch exitflag
        case 1
            return
        case 0
            warning('Timeout: Max iterations reached!');
        case -1
            warning('Infeasible problem!');
        case -2
            warning('Out of memory!');
        case -5
            warning('Error occured during matrix factorization!');
        case -6
            warning('NaN or INF occured during functions evaluations!');
        case -7
            warning('The convex solver could not proceed due to stalled line search!');
        case -10
            warning('The convex solver could not proceed due to an internal error.!');
        case -11
            warning('MPC: Invalid values in problem parameters!');
        case -100
            warning('License error!');
        otherwise
            warning('exitflag not found.')
    end
    
    msg = 1;

end