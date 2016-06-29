% A measure of acceleration capability from static equilibrium
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Implementation of the SEACM
classdef SEACM < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        % Constructor
        function m = SEACM()
            m.metricMin = -Inf;
            m.metricMax = Inf;
        end
        
        % Evaluate Functions implementation
        function v = evaluateFunction(~,dynamics,~)
            L   =   transpose(dynamics.M\dynamics.L_active');
            f_u =   dynamics.cableForcesActiveMax;
            f_l =   dynamics.cableForcesActiveMin;
            w   =   WrenchSet(L,f_u,f_l,dynamics.M\dynamics.G);
            q   =   length(w.b);
            s   =   zeros(q,1);
            for j=1:length(w.b)
                s(j) = (w.b(j) - w.A(j,:)*(dynamics.M\(dynamics.G + dynamics.L_passive.'*dynamics.cableForcesPassive)))/norm(w.A(j,:),2);
            end
            v = min(s);
        end
    end
end