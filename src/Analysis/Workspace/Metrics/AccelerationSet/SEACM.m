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
                w   =   WrenchSet(L,f_u,f_l);
                q   =   w.n_faces;
                s   =   zeros(q,1);
                for j=1:q
                    s(j) = (w.b(j) - w.A(j,:)*(dynamics.M\dynamics.G))/norm(w.A(j,:),2);
                end
                v = min(s);
                % Code for debugging
                %             hold on
                %             g = dynamics.M\dynamics.G;
                %             plot(g(1),g(2),'gx')
                %             t = 0:pi/100:2*pi;
                %             for i=1:length(t)
                %                 x_c(:,i) = (g + v*[cos(t(i));sin(t(i))]).';
                %             end
                %             plot(x_c(1,:),x_c(2,:),'m')
            else
                v = inWorkspace;
            end
        end
    end
end