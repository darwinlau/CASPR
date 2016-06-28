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
    end

    methods
        % The constructor function for minimising the optimally safe 2 norm
        function o = IDObjectiveQuadOptimallySafe(weights)
            o.weights = weights;
            % This is because the general form is
            % (1/2) x^T A x + b^T x + c
            o.A = 2*diag(weights);
            o.b = zeros(length(weights), 1);
            o.c = 0;
        end

        % The objective update implementation
        function updateObjective(obj, dynamics)
            f_m     =   0.5*(dynamics.cableDynamics.forcesMin + dynamics.cableDynamics.forcesMax);
            obj.b   =   -2*diag(obj.weights)*f_m;
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
            obj.A = 2*diag(weights);
            obj.b = zeros(length(weights), 1);
        end
    end
end
