classdef NullMetric < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
        end
    
    methods
        %% Constructor
        function m = NullMetric()
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,~,~,~,~)
            v = 1;
        end
    end
end