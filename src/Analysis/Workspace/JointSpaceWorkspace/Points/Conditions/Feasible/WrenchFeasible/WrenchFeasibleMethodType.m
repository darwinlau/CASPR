% An enum class of for static workspace methods
% Author         : Jonathan EDEN
% Created        : 2015
% Description    : Enum class for different wrench feasible workspace methods
classdef WrenchFeasibleMethodType
    enumeration 
        M_CAPACITY_MARGIN
        M_LINEAR_PROGRAMMING_MATLAB
        M_LINEAR_PROGRAMMING_CPLEX
    end
end