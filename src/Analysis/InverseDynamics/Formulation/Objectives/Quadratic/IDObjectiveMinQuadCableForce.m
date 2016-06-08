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
        % The constructor function for minimising the cable forces
        function o = IDObjectiveMinQuadCableForce(weights)
            o.weights = weights;
            % This is because the general form is
            % (1/2) x^T A x + b^T x + c
            o.A = 2*diag(weights);
            o.b = zeros(length(weights), 1);
            o.c = 0;
        end
        
        % The objective update implementation
        function updateObjective(~, ~)
        end
        
        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A = 2*diag(weights);
            obj.b = zeros(length(weights), 1);
        end
    end    
end

