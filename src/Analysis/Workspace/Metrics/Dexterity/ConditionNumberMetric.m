% The condition number for usage as a dexterity measure.  This doesn't
% consider unilateral actuation constraints but can be used as a point of
% comparison.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : 
classdef ConditionNumberMetric < WorkspaceMetric
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        % Constructor
        function m = ConditionNumberMetric()
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(~,dynamics,~)
            % Determine the Jacobian Matrix
            L = dynamics.L;
            % Compute singular values of jacobian matrix
            Sigma = svd(-L');
            % Compute the condition number
            v = min(Sigma)/max(Sigma);
        end
    end
end