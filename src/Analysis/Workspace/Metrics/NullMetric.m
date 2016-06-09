% Testing class for metrics
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description   : Dummy implementation of abstract functions
classdef NullMetric < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
        end
    
    methods
        % Constructor
        function m = NullMetric()
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(~,~,~,~,~)
            v = 1;
        end
    end
end