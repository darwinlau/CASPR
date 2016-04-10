% Objective function for infinity norm of cable forces to achieve an
% optimally safe cable force distribution
% 
% Author        : Jonathan EDEN
% Created       : 2016
% Description	: The linear coefficients are the weights for each cable
classdef IDObjectiveInfOptimallySafe < IDObjectiveInfinity
    
    properties (SetAccess = protected)
        weights 
    end
    
    methods
        function o = IDObjectiveInfOptimallySafe(weights)
            o.weights = weights;
            o.A = diag(weights);
            o.b = zeros(length(weights),1);
        end
        
        function updateObjective(obj, dynamics)
            f_m     =   0.5*(dynamics.cableDynamics.forcesMin + dynamics.cableDynamics.forcesMax);
            obj.b   =   -diag(obj.weights)*f_m;
        end
        
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A = diag(weights);
            obj.b = zeros(length(weights),1);
        end
    end
end

