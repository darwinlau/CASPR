classdef CapacityMarginMetric < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
        desired_wrench_set
    end
    
    methods
        %% Constructor
        function m = CapacityMarginMetric(desired_wrench_set)
            m.desired_wrench_set = desired_wrench_set;
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics,~,method,inWorkspace)
            if((nargin <=3)||(~(method==WorkspaceStaticMethods.CMa)))
                L   =   dynamics.L;
                f_u =   dynamics.cableDynamics.forcesMax;
                f_l =   dynamics.cableDynamics.forcesMin;
                w   =   WrenchSet(L,f_u,f_l);
                q   =   length(w.b);
                p   =   size(obj.desired_wrench_set,2);
                s   =   zeros(p,q);
                for i = 1:p
                    for j=1:q
                        s(i,j) = (w.b(j) - w.A(j,:)*obj.desired_wrench_set(:,i))/norm(w.A(j,:),2);
                    end
                end
                v = min(min(s));
            else
                v = inWorkspace;
            end
        end
    end
end