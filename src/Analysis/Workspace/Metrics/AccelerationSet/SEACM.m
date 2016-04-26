classdef SEACM < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        %% Constructor
        function m = SEACM()
        end
        
        %% Evaluate Functions
        function v = evaluateFunction(obj,dynamics,~,~,~)
            L   =   transpose(dynamics.M\dynamics.L');
            f_u =   dynamics.forcesMax;
            f_l =   dynamics.forcesMin;
            %                 w   =   WrenchSet(L,f_u,f_l,dynamics.M\dynamics.G);
            w   =   WrenchSet(L,f_u,f_l);
            q   =   length(w.b);
            s   =   zeros(q,1);
            for j=1:length(w.b)
                s(j) = (w.b(j) - w.A(j,:)*(dynamics.M\dynamics.G))/norm(w.A(j,:),2);
            end
            v = min(s);
        end
    end
end