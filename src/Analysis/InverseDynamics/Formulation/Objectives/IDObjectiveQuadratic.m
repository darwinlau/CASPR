% Base quadratic cost function class for inverse dynamics
%
% Author        : Darwin LAU
% Created       : 2016
% Description	: The quadratic function for ID is of the form:
%                   (1/2) x^T A x + b^T x + c
% where x is the variable, A is the Hessian matrix, b is the vector of
% weights on x and c is a constant
classdef IDObjectiveQuadratic < IDObjective
    properties (SetAccess = protected)
        A
        b
        c
    end

    methods
        % Implementatino of the evaluation function for the 2 norm.
        function [f, grad_f] = evaluateFunction(obj, x)
            f = (1/2) * x.' * obj.A * x + obj.b.' * x + obj.c;
            grad_f = (1/2) * (obj.A' + obj.A) * x + obj.b;
        end
    end
end
