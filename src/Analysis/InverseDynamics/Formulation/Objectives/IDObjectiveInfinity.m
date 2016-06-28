% Base infinity norm cost function class for inverse dynamics
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description	: The function for ID is of the form:
%                   min(A*(x+b))
classdef IDObjectiveInfinity < IDObjective
    properties (SetAccess = protected)
        A
        b
    end

    methods
        % Implementation of the evaluate function for infinite norms.
        function [f, grad_f] = evaluateFunction(obj, x)
            f = min(obj.A*(x + obj.b));
            % Gradient not well defined for this case
            grad_f = [];
        end
    end
end
