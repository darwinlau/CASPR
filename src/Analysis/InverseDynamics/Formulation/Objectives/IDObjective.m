% Base objective function with abstract methods defined
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: 
classdef IDObjective < handle
    
    properties
    end
    
    methods (Abstract)
        updateObjective(obj, dynamics);
        [f, grad_f] = evaluateFunction(obj, x);
    end
    
end

