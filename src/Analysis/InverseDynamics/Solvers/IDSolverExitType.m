% Enumeration for the exit type (code) for the inverse dynamics
%
% Author        : Darwin LAU
% Created       : 2014
% Description   : 
classdef IDSolverExitType
    enumeration 
        NO_ERROR
        INFEASIBLE
        ITERATION_LIMIT_REACHED
        SOLVER_SPECIFIC_ERROR
        OTHER_ERROR
    end
end
