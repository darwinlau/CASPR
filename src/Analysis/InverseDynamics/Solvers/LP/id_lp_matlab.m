function [ x_opt, exit_type, comp_time ] = id_lp_matlab(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0)   
    options = optimoptions('linprog', 'Display', 'off');
    start_tic = tic;
    [x_opt, ~, exitflag] = linprog(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
    comp_time = toc(start_tic);
    switch exitflag
        case 1
            exit_type = IDSolverExitType.NO_ERROR;
        case 0
            fprintf('Max iteration limit reached\n');
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -2
            fprintf('Problem infeasible\n');
            exit_type = IDSolverExistType.INFEASIBLE;
        otherwise
            fprintf('Other error : Code %d\n', exit_flag);
            exit_type = IDSolverExistType.SOLVER_SPECIFIC_ERROR;
    end
end