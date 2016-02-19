% Base quadratic cost function class for inverse dynamics
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: The quadratic function for ID is of the form:
%                   min(A*(x+b))
% where x is the variable, A is the Hessian matrix, b is the vector of 
% weights on x and c is a constant
classdef IDObjectiveInfinity < IDObjective
    
    properties (SetAccess = protected)
        A
        b
    end
    
    methods
        function [f, grad_f] = evaluateFunction(obj, x)
            f = min(obj.A*(x + obj.b));
            % Gradient not well defined for this case
            grad_f = [];
        end
    end
    
end

