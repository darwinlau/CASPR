classdef MagnitudeVelocityMetric < Metric
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        %% Constructor
        function m = MagnitudeVelocityMetric()
            m.options   =    	optimset('display','off');
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics)
            L   =   dynamics.L;
            f_u =   dynamics.cableDynamics.forcesMax;
            f_l =   dynamics.cableDynamics.forcesMin;
            w   =   WrenchSet(L,f_u,f_l);
            q   =   w.n_faces;
            s   =   zeros(q,1);
            q2 = dynamics.q(2);
            t = sign(sin(q2));
            for j=1:q
                s(j) = (w.b(j) - w.A(j,:)*dynamics.G)/norm(w.A(j,:));
                p = dynamics.G + s(j)*(w.A(j,:)')/norm(w.A(j,:));
                if((t*(p(2)-dynamics.G(2))<0)&&(sign(s(j))>0))
                    pd = (1/w.A(j,1))*(w.b(j) - w.A(j,2)*dynamics.G(2));
                    if(abs(pd)==Inf)
                        s(j) = Inf;
                    else
                        s(j) = abs(pd - dynamics.G(1));
                    end
                    % Find the correct intersection of the boundary ray to the line
                end
            end
            mv = min(s);
            a = 0.5*sin(q2);
            % For the moment this assumes 2R planar manipulator
            if(norm(a)<1e-4)
                v = sign(mv)*Inf;
            else
                v = sign(mv)*(mv^2/(3*a^2))^(0.25);
            end
        end
        
        function v = workspaceCheck(obj,type)
            if((type == WorkspaceType.SW) || (type == WorkspaceType.SCW))
                v = 1;
            else
                v = 0;
            end
        end
    end
end