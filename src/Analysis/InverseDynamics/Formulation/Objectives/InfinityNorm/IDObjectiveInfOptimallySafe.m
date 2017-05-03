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
            CASPR_log.Assert(length(obj.weights) == dynamics.numActuators, 'Dimensions of weight is not correct, it should be a vector of length numActuators');
            indices_all = 1:dynamics.numActuators;
            indices_active = indices_all(setdiff(1:length(indices_all), dynamics.cableModel.cableIndicesPassive));
            obj.A   = obj.A_full(indices_active, indices_active);
            f_m     = 0.5*(dynamics.actuationForcesMin + dynamics.actuationForcesMax);
            obj.b   = -diag(obj.weights(indices_active)) * f_m;
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A_full = diag(weights);
            obj.b_full = zeros(length(weights),1);
        end
    end
end