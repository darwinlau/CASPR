% Base objective function with abstract methods defined
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: 
classdef IDObjective < handle
    
    properties
    end
    
    methods (Abstract)
        updateParameters(obj, dynamics);
        f = evaluateFunction(obj, x);
    end
    
end

