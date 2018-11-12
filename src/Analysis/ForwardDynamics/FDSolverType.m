% Enum of the types of solvers for the forward dynamics integration
%
% Author        : Darwin LAU
% Created       : 2016
% Description    : 
classdef FDSolverType
    enumeration
        % Variable step solvers
        ODE45
        ODE23
        ODE113
        ODE15S
        ODE23S
        ODE23T
        ODE23TB
        % Fixed step solvers (no error control mechanism)
        % Fourth-Order Runge-Kutta (RK4)
        ODE4
        % Euler's method
        ODE1
    end
end