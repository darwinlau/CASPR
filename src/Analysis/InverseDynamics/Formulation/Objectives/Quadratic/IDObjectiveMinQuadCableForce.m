% Objective function for quadratic sum of cable forces
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: The weights are the diagonals of the Hessian matrix (A)
classdef IDObjectiveMinQuadCableForce < IDObjectiveQuadratic
    
    properties (SetAccess = protected)
        weights
    end
    
    methods
        function o = IDObjectiveMinQuadCableForce(weights)
            o.weights = weights;
        end
        
        function updateObjective(obj, ~)
            % This is because the general form is
            % (1/2) x^T A x + b^T x + c
            obj.A = 2*diag(obj.weights);
            obj.b = zeros(length(obj.weights), 1);
            obj.c = 0;
        end
        
        function updateWeights(obj, weights)
            obj.weights = weights;
        end
    end    
end

