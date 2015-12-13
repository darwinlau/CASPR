classdef SEACM < WorkspaceMetric
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        %% Constructor
        function m = SEACM()
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics,~,method,inWorkspace)
            if((nargin <=3)||(~(method==StaticMethods.CMe)))
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
                if(sum(s<-1e-6)>0)
                    min(s)
                    disp('more fail')
                    sfgdkjsjfhd
                end
                v = min(s);
%                 sdh
                %Code for debugging
%                 hold on
% %                 g = dynamics.M\dynamics.G;
%                 plot(0,0,'r.')
%                 t = 0:pi/100:2*pi;
%                 for i=1:length(t)
%                     x_c(:,i) = (v*[cos(t(i));sin(t(i))]).';
%                 end
%                 plot(x_c(1,:),x_c(2,:),'m')
%                 xlabel('w_1 (Nm)')
%                 ylabel('w_2 (Nm)')
            else
                v = inWorkspace;
            end
        end
    end
end