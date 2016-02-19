% Objective function for infinity norm of cable forces
% 
% Author        : Jonathan EDEN
% Created       : 2016
% Description	: The linear coefficients are the weights for each cable
classdef IDObjectiveMinInfCableForce < IDObjectiveInfinity
    
    properties (SetAccess = protected)
        weights 
    end
    
    methods
        function o = IDObjectiveMinInfCableForce(weights)
            o.weights = weights;
            o.A = diag(weights);
            o.b = zeros(length(weights),1);
        end
        
        function updateObjective(~, ~)
        end
        
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A = diag(weights);
            obj.b = zeros(length(weights),1);
        end
    end
end

