% An efficient inverse dynamics solver for CDPR systems.
%
% Please cite the following paper when using this algorithm:
% A. Pott and T. Bruckmann and L. Mikelsons "Closed-form force
% distribution for parallel wire robots". In Computational kinematics,
% pp 25-34. Springer, Berlin, 2009.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : This is an implementation of the original version of the
%                 closed form method.
function [ x_opt, exit_type ] = id_cf_cfm(A_eq, b_eq, x_min, x_max)
    x_m = 0.5*(x_min + x_max);
    x_opt = x_m + pinv(A_eq)*(b_eq - A_eq*x_m);
    if((sum(x_opt - x_min < -1e-6)>0)||(sum(x_opt - x_max > 1e-6)>0))
        % Solution is not feasible
        exit_type = IDSolverExitType.INFEASIBLE;
    else
        exit_type = IDSolverExitType.NO_ERROR;
    end
end
