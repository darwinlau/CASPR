% Objective function for linear sum of cable forces
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: The linear coefficients are the weights for each cable
classdef IDObjectiveMinLinCableForce < IDObjectiveLinear
    
    properties (SetAccess = protected)
        weights 
    end
    
    methods
        function o = IDObjectiveMinLinCableForce(weights)
            o.weights = weights;
        end
        
        function updateParameters(obj, ~)
            obj.b = obj.weights;
            obj.c = 0;
        end
        
        function updateWeights(obj, weights)
            obj.weights = weights;
        end
    end
end

