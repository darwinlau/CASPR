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
            obj.A = obj.A_full(dynamics.cableModel.cableIndicesActive, dynamics.cableModel.cableIndicesActive);
            obj.b = obj.b_full(dynamics.cableModel.cableIndicesActive);
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A_full = diag(weights);
            obj.b_full = zeros(length(weights),1);
        end
    end
end
