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
    
    properties
    end
    
    methods (Abstract)
        updateObjective(obj, dynamics);
        [f, grad_f] = evaluateFunction(obj, x);
    end
    
end

