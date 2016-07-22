% Objective function for quadratic sum of cable forces
%
% Author        : Darwin LAU
% Created       : 2016
% Description	: The weights are the diagonals of the Hessian matrix (A)
classdef IDObjectiveMinQuadCableForce < IDObjectiveQuadratic
    properties (SetAccess = protected)
        weights
        A_full
        b_full
    end

    methods
        % The constructor function for minimising the cable forces
        function o = IDObjectiveMinQuadCableForce(weights)
            o.updateWeights(weights);
            o.c = 0;
        end

        % The objective update implementation
        function updateObjective(obj, dynamics)            
            CASPR_log.Assert(length(obj.weights) == dynamics.numActuators, 'Dimensions of weight is not correct, it should be a vector of length numActuators');
            
            indices_all = 1:dynamics.numActuators;
            indices_active = indices_all(setdiff(1:length(indices_all), dynamics.cableModel.cableIndicesPassive));
            % Only select the active cable elements to consider
            obj.A = obj.A_full(indices_active, indices_active);
            obj.b = obj.b_full(indices_active);
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
            % This is because the general form is
            % (1/2) x^T A x + b^T x + c
            obj.A_full = 2*diag(weights);
            obj.b_full = zeros(length(weights), 1);
        end
    end
end
