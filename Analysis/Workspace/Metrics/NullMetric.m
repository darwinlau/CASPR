classdef NullMetric < Metric
    properties (SetAccess = protected, GetAccess = protected)
        end
    
    methods
        %% Constructor
        function m = NullMetric()
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics)
            v = 0;
        end
        
        function v = workspaceCheck(obj,type)
            v = 0;
        end
    end
end