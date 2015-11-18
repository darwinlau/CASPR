classdef CapabilityMeasureMetric < WorkspaceMetric
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        %% Constructor
        function m = CapabilityMeasureMetric()
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics)
            L   =   transpose(dynamics.M\dynamics.L');
            f_u =   dynamics.cableDynamics.forcesMax;
            f_l =   dynamics.cableDynamics.forcesMin;
            w   =   WrenchSet(L,f_u,f_l);
            q   =   w.n_faces;
            s   =   zeros(q,1);
            for j=1:q
                s(j) = (w.b(j) - w.A(j,:)*(dynamics.M\dynamics.G))/norm(w.A(j,:));
            end
            v = min(s);
        end
    end
end