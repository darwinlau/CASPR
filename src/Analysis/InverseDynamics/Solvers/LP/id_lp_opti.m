% Implementation of linear programming using the optitollbox
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Optitoolbox LP implementation
function [ x_opt, exit_type ] = id_lp_opti(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0,opts) 
    [x_opt, ~, exitflag] = opti_linprog(b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, opts);
    
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