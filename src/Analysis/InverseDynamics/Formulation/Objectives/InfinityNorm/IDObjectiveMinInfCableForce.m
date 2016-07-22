% Objective function for infinity norm of cable forces
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description	: The linear coefficients are the weights for each cable
classdef IDObjectiveMinInfCableForce < IDObjectiveInfinity
    properties (SetAccess = protected)
        weights
        A_full
        b_full
    end

    methods
        % The constructor function for minimising the infinite cable force
        % norms.
        function o = IDObjectiveMinInfCableForce(weights)
            o.updateWeights(weights);
        end

        % The objective update implementation
        function updateObjective(obj, dynamics)
            CASPR_log.Assert(length(obj.weights) == dynamics.numActuators, 'Dimensions of weight is not correct, it should be a vector of length numActuators');
            
            indices_all = 1:dynamics.numActuators;
            indices_active = indices_all(setdiff(1:length(indices_all), dynamics.cableModel.cableIndicesPassive));
            obj.A = obj.A_full(indices_active, indices_active);
            obj.b = obj.b_full(indices_active);
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A_full = diag(weights);
            obj.b_full = zeros(length(weights),1);
        end
    end
end
