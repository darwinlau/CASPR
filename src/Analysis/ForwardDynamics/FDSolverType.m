% Enum of the types of solvers for the forward dynamics integration
%
% Author        : Darwin LAU
% Created       : 2016
% Description    : 
classdef FDSolverType
    enumeration 
        ODE45
        ODE23
        ODE113
        ODE15S
        ODE23S
        ODE23T
        ODE23TB
        ODE4
    end
end