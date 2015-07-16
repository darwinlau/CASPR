classdef RelativeRadiusMetric < Metric
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
        ref
    end
    
    methods
        %% Constructor
        function m = RelativeRadiusMetric(ref)
            m.options   =    	optimset('display','off');
            m.ref       =       ref;
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics)
            L       =   dynamics.L;
            f_u     =   dynamics.cableDynamics.forcesMax;
            f_l     =   dynamics.cableDynamics.forcesMin;
            w       =   WrenchSet(L,f_u,f_l);
            w_ch    =	w.sphereApproximationChebyshev();
            w_sp    =   w.sphereApproximationCapacity(obj.ref);
            v       =   w_sp.r/w_ch.r;
        end
        
        function v = workspaceCheck(obj,type)
            if(type == WorkspaceType.WCW)
                v = 1;
            else
                v = 0;
            end
        end
    end
end