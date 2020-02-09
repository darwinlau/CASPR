% A measure of acceleration capability from static equilibrium
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Implementation of the SEACM
classdef SEACMetric < WorkspaceMetricBase
    % Constants that needs to be defined from parent
    properties (Constant)
        type = WorkspaceMetricType.SEACM;
        metricMin = -Inf;
        metricMax = Inf;
    end
    
    methods
        % Constructor
        function m = SEACMetric()
        end
        
        % Evaluate Functions implementation
        function v = evaluateFunction(~, dynamics)
            a = dynamics.availStaticAccelerationSet;
            if(a.n_faces > 0)
                q   =   length(a.b);
                s   =   zeros(q,1);
                for j=1:length(a.b)
                    s(j) = a.b(j)/norm(a.A(j,:),2);
                end
                v = min(s);
            else
                v = 0;
            end
        end
    end
end