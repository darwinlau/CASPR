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
        function f = evaluateFunction(obj, x)
            f = obj.b.'*x + obj.c;
        end
    end
    
end

