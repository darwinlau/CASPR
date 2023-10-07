% Implementation of linear programming using the default MATLAB solvers.
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : MATLAB LP implementation
function [ x_opt, exit_type ] = id_lp_matlab(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax,options)
    [x_opt, ~, exitflag] = linprog(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, options);
    switch exitflag
        case 1
            exit_type = IDSolverExitType.NO_ERROR;
        case 0
            CASPR_log.Info('Max iteration limit reached');
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -2
            CASPR_log.Info('Problem infeasible');
            exit_type = IDSolverExitType.INFEASIBLE;
            x_opt = xmin;
        otherwise
            CASPR_log.Info(sprintf('Other error : Code %d', exitflag));
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
    end
end
