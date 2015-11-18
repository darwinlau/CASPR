classdef RelativeVolumeMetric < Metric
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        %% Constructor
        function m = RelativeVolumeMetric()
            m.options   =    	optimset('display','off');
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics)
            L       =   dynamics.L;
            f_u     =   dynamics.cableDynamics.forcesMax;
            f_l     =   dynamics.cableDynamics.forcesMin;
            w       =   WrenchSet(L,f_u,f_l);
            w_ch    =	w.sphereApproximationChebyshev();
            v       =   (pi*w_ch.r^2)/w.v;
        end
        
        function v = workspaceCheck(obj,type)
            v = 0;
        end
    end
end