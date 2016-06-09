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
        end
        
        % Evaluate Functions implementation
        function v = evaluateFunction(obj,dynamics,~,~,~)
            L   =   transpose(dynamics.M\dynamics.L');
            f_u =   dynamics.forcesMax;
            f_l =   dynamics.forcesMin;
            %                 w   =   WrenchSet(L,f_u,f_l,dynamics.M\dynamics.G);
            w   =   WrenchSet(L,f_u,f_l,dynamics.M\dynamics.G);
            
            q   =   length(w.b);
            s   =   zeros(q,1);
            for j=1:length(w.b)
                s(j) = (w.b(j) - w.A(j,:)*(dynamics.M\dynamics.G))/norm(w.A(j,:),2);
            end
            v = min(s);
%             hold on
%             plot(0,0,'r.')
%             t = 0:pi/100:2*pi;
%             for i=1:length(t)
%                 x_c(:,i) = (v*[cos(t(i));sin(t(i))]).';
%             end
%             plot(x_c(1,:),x_c(2,:),'m')
%             xlabel('$\ddot{q}_1$ (rad/s$^2$)','interpreter','latex')
%             ylabel('$\ddot{q}_2$ (rad/s$^2$)','interpreter','latex')
%             grid on
%             axis([-1500,1500,-1500,1500])
        end
    end
end