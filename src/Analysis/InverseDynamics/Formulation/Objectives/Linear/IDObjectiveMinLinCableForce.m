% Objective function for linear sum of cable forces
%
% Author        : Darwin LAU
% Created       : 2016
% Description	: The linear coefficients are the weights for each cable
classdef IDObjectiveMinLinCableForce < IDObjectiveLinear
    properties (SetAccess = protected)
        weights
        A_full
        b_full
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
            obj.b = obj.weights(dynamics.cableModel.cableIndicesActive);
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
        end
    end
end
