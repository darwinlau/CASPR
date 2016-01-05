classdef CapacityMarginMetric < WorkspaceMetric
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        %% Constructor
        function m = CapacityMarginMetric()
            m.options   =    	optimset('display','off');
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics,~,method,inWorkspace)
            if((nargin <=3)||(~(method==WorkspaceStaticMethods.CMa)))
                L   =   dynamics.L;
                f_u =   dynamics.cableDynamics.forcesMax;
                f_l =   dynamics.cableDynamics.forcesMin;
                w   =   WrenchSet(L,f_u,f_l);
                q   =   length(w.b);
                s   =   zeros(q,1);
                for j=1:q
                    s(j) = (w.b(j) - w.A(j,:)*dynamics.G)/norm(w.A(j,:),2);
                end
                v = min(s);
            else
                v = inWorkspace;
            end
        end
    end
end