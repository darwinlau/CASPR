% Objective function for the optimally safe distribution, defined as aiming
% to produce cable forces that are in the middle of the minimum and maximum
% of cable forces
% 
% Author        : Jonathan EDEN
% Created       : 2016
% Description	: 
classdef IDObjectiveQuadOptimallySafe < IDObjectiveQuadratic
    
    properties (SetAccess = protected)
        weights
    end
    
    methods
        function o = IDObjectiveQuadOptimallySafe(weights)
            o.weights = weights;
            % This is because the general form is
            % (1/2) x^T A x + b^T x + c
            o.A = 2*diag(weights);
            o.b = zeros(length(weights), 1);
            o.c = 0;
        end
        
        function updateObjective(obj, dynamics)
            f_m     =   0.5*(dynamics.cableDynamics.forcesMin + dynamics.cableDynamics.forcesMax);
            obj.b   =   -2*diag(obj.weights)*f_m;
        end
        
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A = 2*diag(weights);
            obj.b = zeros(length(weights), 1);
        end
    end    
end

