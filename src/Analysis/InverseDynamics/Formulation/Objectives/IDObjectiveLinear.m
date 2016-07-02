% Base linear cost function class for inverse dynamics
%
% Author        : Darwin LAU
% Created       : 2016
% Description	: The linear function for ID is of the form b^T*x + c,
% where x is the variable, b is the vector of weights on x and c is a
% constant
classdef IDObjectiveLinear < IDObjective
    properties (SetAccess = protected)
        b
        c
    end

    methods
        % Implementation of the evaluation function for the 1 norm.
        function [f, grad_f] = evaluateFunction(obj, x)
            f = obj.b.'*x + obj.c;
            grad_f = obj.b;
        end
    end
end
