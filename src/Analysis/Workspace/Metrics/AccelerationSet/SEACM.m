classdef SEACM < WorkspaceMetric
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        %% Constructor
        function m = SEACM()
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics,~,method,inWorkspace)
            if((nargin <=3)||(~(method==WorkspaceStaticMethods.CMe)))
                L   =   transpose(dynamics.M\dynamics.L');
                f_u =   dynamics.cableDynamics.forcesMax;
                f_l =   dynamics.cableDynamics.forcesMin;
%                 w   =   WrenchSet(L,f_u,f_l,dynamics.M\dynamics.G);
                w   =   WrenchSet(L,f_u,f_l);
                q   =   length(w.b);
                s   =   zeros(q,1);
                for j=1:length(w.b)
                    s(j) = (w.b(j) - w.A(j,:)*(dynamics.M\dynamics.G))/norm(w.A(j,:),2);
                end
                v = min(s);
            else
                v = inWorkspace;
            end
        end
    end
end