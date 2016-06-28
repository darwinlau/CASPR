% Base objective function with abstract methods defined
%
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%   The general form of the objective function is
%                       y = f(x),       output y and input x
%   The objective function has two main functions:
%       1) Update the parameters within the objective using the system
%       dynamics of the CDPR
%       2) Evaluate the objective by returning the value of f and the
%       gradient of f
classdef IDObjective < handle
    methods (Abstract)
        % Update the objective to consider the dynamics.
        updateObjective(obj, dynamics);
        % The evaluation function converts f into an objective value and
        % its gradient.
        [f, grad_f] = evaluateFunction(obj, x);
    end
end
