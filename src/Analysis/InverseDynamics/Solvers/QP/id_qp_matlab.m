% The MATLAB implementation of quadratic programming solvers
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : MATLAB implementation of QP solvers
function [ x_opt, exit_type ] = id_qp_matlab(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0,options)
%     options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
    [x_opt, ~, exitflag] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
    switch exitflag
        case 1
            exit_type = IDSolverExitType.NO_ERROR;
        case 0
            CASPR_log.Print('Max iteration limit reached\n',CASPRLogLevel.INFO);
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -2
            CASPR_log.Print('Problem infeasible\n',CASPRLogLevel.INFO);
            exit_type = IDSolverExitType.INFEASIBLE;
            x_opt = xmin;
        otherwise
            CASPR_log.Print(sprintf('Other error : Code %d\n', exit_flag),CASPRLogLevel.INFO);
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
    end
end
