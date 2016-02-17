function [ x_opt, exit_type ] = id_lp_optitoolbox_clp(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0)    
    opts = optiset('solver', 'CLP', 'maxiter', 1000);
    optisolver = opti('f', b, 'ineq', A_ineq, b_ineq, 'eq', A_eq, b_eq, 'bounds', xmin, xmax, 'options', opts);
    [x_opt, ~, exitflag] = solve(optisolver,x0);
    %[x_opt, ~, exitflag, id_info] = solve(optisolver, x0);
    %comp_time = id_info.Time;
    
    switch exitflag
        case 1
            exit_type = IDSolverExitType.NO_ERROR;
        case -1
            fprintf('Problem infeasible\n');
            exit_type = IDSolverExitType.INFEASIBLE;
        case 0
            fprintf('Max iteration limit reached\n');
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -3
            fprintf('Solver specific error\n');
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
        otherwise
            fprintf('Other error : Code %d\n', exit_flag);
            exit_type = IDSolverExitType.OTHER_ERROR;
    end
end