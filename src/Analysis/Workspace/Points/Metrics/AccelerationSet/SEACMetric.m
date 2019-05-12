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
            L   =   transpose(dynamics.M\dynamics.L_active');
            f_u =   dynamics.actuationForcesMax;
            f_l =   dynamics.actuationForcesMin;
            w   =   WrenchSet(L,f_u,f_l,dynamics.M\dynamics.G);
            if(w.n_faces > 0)
                q   =   length(w.b);
                s   =   zeros(q,1);
                for j=1:length(w.b)
                    s(j) = (w.b(j) - w.A(j,:)*(dynamics.M\(dynamics.G + dynamics.L_passive.'*dynamics.cableForcesPassive)))/norm(w.A(j,:),2);
                end
                v = min(s);
            else
                v = 0;
            end
        end
    end
end