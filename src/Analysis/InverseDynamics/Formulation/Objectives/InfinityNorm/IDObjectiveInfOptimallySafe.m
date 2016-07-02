% Objective function for infinity norm of cable forces to achieve an
% optimally safe cable force distribution
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description	: The linear coefficients are the weights for each cable
classdef IDObjectiveInfOptimallySafe < IDObjectiveInfinity
    properties (SetAccess = protected)
        weights
        A_full
        b_full
    end

    methods
        % The constructor function for optimally safe infinite norms.
        function o = IDObjectiveInfOptimallySafe(weights)
            o.updateWeights(weights);
        end

        % The objective update implementation
        function updateObjective(obj, dynamics)
            obj.A   = obj.A_full(dynamics.cableModel.cableIndicesActive, dynamics.cableModel.cableIndicesActive);
            f_m     = 0.5*(dynamics.cableForcesActiveMin + dynamics.cableForcesActiveMax);
            obj.b   = -diag(obj.weights(dynamics.cableModel.cableIndicesActive)) * f_m;
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A_full = diag(weights);
            obj.b_full = zeros(length(weights),1);
        end
    end
end
