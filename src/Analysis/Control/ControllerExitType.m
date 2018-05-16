% Enumeration for the exit type (code) for the controllers using its own
% optimization formulation instead of Inverse Dynamics solver
%
% Author        : Chen SONG
% Created       : 2018
% Description   : 
classdef ControllerExitType
    enumeration 
        NO_ERROR
        INFEASIBLE
        ITERATION_LIMIT_REACHED
        SOLVER_SPECIFIC_ERROR
        OTHER_ERROR
    end
end
