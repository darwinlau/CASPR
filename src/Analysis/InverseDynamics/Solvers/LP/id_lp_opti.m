% Implementation of linear programming using the optitollbox
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Optitoolbox LP implementation
function [ x_opt, exit_type ] = id_lp_opti(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0,opts)
    [x_opt, ~, exitflag] = opti_linprog(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, opts);
    switch exitflag
        case 1
            exit_type = IDSolverExitType.NO_ERROR;
        case 0
            CASPR_log.Print('Max iteration limit reached\n',CASPRLogLevel.INFO);
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -1
            CASPR_log.Print('Problem infeasible\n',CASPRLogLevel.INFO);
            exit_type = IDSolverExitType.INFEASIBLE;
            x_opt = xmin;
        case -3
            CASPR_log.Print('Solver specific error\n',CASPRLogLevel.INFO);
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
        otherwise
            CASPR_log.Print(sprintf('Other error : Code %d\n', exit_flag),CASPRLogLevel.INFO);
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
    end
end
