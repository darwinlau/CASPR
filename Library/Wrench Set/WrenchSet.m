classdef WrenchSet < handle
    % WrenchSet Class to store information about the wrench set and 
        
    properties (SetAccess = protected)
        n_faces
        A
        b
        v
    end
    
    methods
        function id = WrenchSet(L,f_u,f_l)
            n = size(L,2); m = size(L,1);
            q = 2^m;
            f = zeros(q,m);
            w = zeros(q,n);
            for k=1:q
                % Convert k to binary
                beta = double(dec2bin(k-1,m)) - 48;
                f(k,:) = (ones(m) - diag(beta))*f_l + diag(beta)*f_u;
                w(k,:) = -L'*f(k,:)';
            end
            [K,id.v] = convhulln(w);
            id.n_faces = size(K,1);
            q = id.n_faces;
            t_A = zeros(q,n);
            t_b = zeros(q,1);
            for i = 1:q
                g = (w(K(i,1),2)-w(K(i,2),2))/(w(K(i,1),1)-w(K(i,2),1));
                t_b(i) = (-g*w(K(i,1),1) + w(K(i,1),2));
                t_A(i,:) = [-g,1];
                if(i==1)
                    if(t_A(i,:)*w(K(q,1),:)'>t_b(i))
                        t_A(i,:) = -t_A(i,:);
                        t_b(i) = -t_b(i);
                    end
                else
                    if(t_A(i,:)*w(K(i-1,1),:)'>t_b(i))
                        t_A(i,:) = -t_A(i,:);
                        t_b(i) = -t_b(i);
                    end
                end
            end
            id.A = t_A;
            id.b = t_b;
        end
        
        function w_approx_sphere = sphereApproximationCapacity(obj,G)
            q = obj.n_faces;
            s = zeros(q,1);
            for j=1:q
                s(j) = (obj.b(j) - obj.A(j,:)*G)/norm(obj.A(j,:));
            end
            s = min(s);
            w_approx_sphere = WrenchSetSphere(G,s);
        end
        
        function w_approx_sphere = sphereApproximationChebyshev(obj)
            q = obj.n_faces;
            n = size(obj.A,2);
            % Check Chebyshev centre
            A_c = [obj.A,zeros(q,1)];
            for i =1:q
                A_c(i,n+1) = norm(obj.A(i,:));
            end
            C = [0,0,1];
            cvx_begin quiet
                variables o(3)
                maximize( C*o)
                subject to
                A_c*o <= obj.b
            cvx_end
            w_approx_sphere = WrenchSetSphere(o(1:2),o(3));
        end
        
        function w_approx_sphere = sphereApproximationMax(obj,x_ref,buffer)
            options = optimoptions('fmincon','Algorithm','active-set','Display','off');
            [T,r,~] = fmincon(@(x) costMinRad(x,obj.A,obj.b),[0;0],obj.A,obj.b,[],[],[],[],@(x) constraintPointContained(x,obj.A,obj.b,x_ref,buffer),options);
            w_approx_sphere = WrenchSetSphere(T,r);
        end
    end
end

