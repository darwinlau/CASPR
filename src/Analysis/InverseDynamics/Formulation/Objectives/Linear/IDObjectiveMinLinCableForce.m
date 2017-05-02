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
        % The constructor function for minimising the 1 cable force
        % norms.
        function o = IDObjectiveMinLinCableForce(weights)
            o.updateWeights(weights);
            o.c = 0;
        end

        % The objective update implementation
        function updateObjective(obj, dynamics)
            CASPR_log.Assert(length(obj.weights) == dynamics.numActuators, 'Dimensions of weight is not correct, it should be a vector of length numActuators');
            indices_all = 1:dynamics.numActuators;
            indices_active = indices_all(setdiff(1:length(indices_all), dynamics.cableModel.cableIndicesPassive));
            obj.b = obj.weights(indices_active);
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
        end
    end
end
