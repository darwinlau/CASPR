% An enum class of for dynamic feasible workspace methods
% Author         : Chen SONG
% Created        : 2019
% Description    : Enum class for different dynamic feasible workspace methods
classdef DynamicFeasibleMethodType
    enumeration 
        M_CONSTANT_VELOCITY_MATLAB
        M_VELOCITY_SET_QUADRATIC_C_MATLAB
        M_VELOCITY_SET_QUADRATIC_C_CPLEX_INTERVAL
    end
end