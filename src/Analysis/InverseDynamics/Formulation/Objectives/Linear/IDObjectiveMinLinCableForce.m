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
            o.weights = weights;
        end

        % The objective update implementation
        function updateObjective(obj, ~)
            obj.b = obj.weights;
            obj.c = 0;
        end

        % An update of the weights
        function updateWeights(obj, weights)
            obj.weights = weights;
        end
    end
end
