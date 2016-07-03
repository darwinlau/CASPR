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
        A_full
        b_full
    end

    methods
        % The constructor function for minimising the optimally safe 2 norm
        function o = IDObjectiveQuadOptimallySafe(weights)
            o.updateWeights(weights);
            o.c = 0;
        end

        % The objective update implementation
        function updateObjective(obj, dynamics)
            CASPR_log.Assert(length(obj.weights) == dynamics.numCables, 'Dimensions of weight is not correct, it should be a vector of length numCables');
            
            f_m     =   0.5*(dynamics.cableForcesActiveMin + dynamics.cableForcesActiveMax);
            obj.A   =   obj.A_full(dynamics.cableModel.cableIndicesActive, dynamics.cableModel.cableIndicesActive);
            obj.b   =   -2*diag(obj.weights(dynamics.cableModel.cableIndicesActive))*f_m;
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
