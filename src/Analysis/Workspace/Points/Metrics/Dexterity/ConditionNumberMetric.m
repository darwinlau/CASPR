% The condition number for usage as a dexterity measure.  This doesn't
% consider unilateral actuation constraints but can be used as a point of
% comparison.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : 
classdef ConditionNumberMetric < WorkspaceMetricBase
    % Constants that needs to be defined from parent
    properties (Constant)
        type = WorkspaceMetricType.CONDITION_NUMBER;
        metricMin = 0;
        metricMax = Inf;
    end
    
    methods
        % Constructor
        function m = ConditionNumberMetric()
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(~, dynamics)
            % Determine the Jacobian Matrix
            L = dynamics.L_active;
            % Compute singular values of jacobian matrix
            Sigma = svd(-L');
            % Compute the condition number
            v = min(Sigma)/max(Sigma);
        end
    end
end