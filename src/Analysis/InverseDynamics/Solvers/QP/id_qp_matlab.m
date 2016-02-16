function [ x_opt, exit_type ] = id_qp_matlab(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0,options)
%     options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
    [x_opt, ~, exitflag] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
    switch exitflag
        case 1
            exit_type = IDSolverExitType.NO_ERROR;
        case 0
            fprintf('Max iteration limit reached\n');
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -2
            fprintf('Problem infeasible\n');
            exit_type = IDSolverExitType.INFEASIBLE;
            x_opt = xmin;
        otherwise
            fprintf('Other error : Code %d\n', exitflag);
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
    end
end

