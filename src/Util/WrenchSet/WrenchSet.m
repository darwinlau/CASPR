% Object representation of the wrench-set
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef WrenchSet < handle
        
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
            % Set up the correct directory for logging
            str = mfilename('fullpath');
            s_id = strfind(str,'mcdm-analysis.matlab');
            str  = str(1:s_id+19);
            str_logs = [str,'\logs'];
            if(~exist(str_logs,'dir'))
                mkdir(str_logs)
            end
            str_io1 = [str_logs,'\wrench_set.txt'];
            fID = fopen(str_io1,'w');
            fprintf(fID,'%d\n',n);
            fprintf(fID,'%d\n',q);
            for k=1:q
                % Convert k to binary
                beta = double(dec2bin(k-1,m)) - 48;
                f(k,:) = (eye(m) - diag(beta))*f_l + diag(beta)*f_u;
                w(k,:) = -L'*f(k,:)';
                fprintf(fID,'%5.12f\t',w(k,:));
                fprintf(fID,'\n');
            end
            fclose(fID);
            % Now offload to qhull
            str_io2 = [str_logs,'\convhull.txt'];
            str_qconvex = ['!',str,'\dependencies\qhull-2012.1\bin\qconvex n Qs < ',str_io1,' > ',str_io2];
            eval(str_qconvex);
%             ! ..\..\dependencies\qhull-2012.1\bin\qconvex n Qs < ..\..\logs\wrench_set.txt > ..\..\logs\convhull.txt
            fID2 = fopen(str_io2);
            % Read the first two lines of the file
            n_s = fgets(fID2);
            n_f = fgets(fID2);
            T = fscanf(fID2,'%f',[n+1,inf])';
            fclose(fID2);
            % Convert to A, b form
            id.A = T(:,1:n);
            id.b = -T(:,n+1);
        end
        
        % External Call Free Version
%         function id = WrenchSet(L,f_u,f_l)
%             n = size(L,2); m = size(L,1);
%             q = 2^m;
%             f = zeros(q,m);
%             w = zeros(q,n);
%             for k=1:q
%                 % Convert k to binary
%                 beta = double(dec2bin(k-1,m)) - 48;
%                 f(k,:) = (eye(m) - diag(beta))*f_l + diag(beta)*f_u;
%                 w(k,:) = -L'*f(k,:)';
%             end
%             [K,id.v] = convhulln(w);
%             
%             id.n_faces = size(K,1);
%             n_shape = size(K,2);
%             n_f = id.n_faces;
%             t_A = zeros(n_f,n);
%             t_b = zeros(n_f,1);
%             count = 1;
%             for i = 1:n_f
%                 W = w(K(i,2:end),:) - repmat(w(K(i,1),:),n_shape-1,1);
%                 T = null(W)';
%                 if(rank(T)>1)
%                 else
%                     t_A(count,:) = T;
%                     t_b(count,:) = t_A(count,:)*w(K(i,1),:).'; 
%                     % Find a new vertext not in the face
%                     s_flag = 0; j = 1;
%                     while(s_flag == 0)
%                         if((sum(j==K(i,:))==0)&&(norm(t_A(count,:)*w(K(j,1),:)'-t_b(count))>1e-6))
%                             s_flag = 1;
%                         else
%                             j = j+1;
%                         end
%                     end
%                     if(t_A(count,:)*w(K(j,1),:)'>t_b(count))
%                         t_A(count,:) = -t_A(count,:);
%                         t_b(count) = -t_b(count);
%                     end
%                     count = count + 1;
%                 end
%             end
%             id.A = t_A(1:count-1,:);
%             id.b = t_b(1:count-1);
%         end
        
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
        
        function w_approx_sphere = sphereApproximationCoriolis(obj,G,q2)
            % THIS NEEDS TO BE CHANGED LATER
            q = obj.n_faces;
            s = zeros(q,1);
            t = sign(sin(q2));
            for j=1:q
                s(j) = (obj.b(j) - obj.A(j,:)*G)/norm(obj.A(j,:));
                p = dynamics.G + s(j)*(w.A(j,:)')/norm(w.A(j,:));
                if(t*(p(2)-dynamics.G(2))<0)
                    pd = (1/w.A(j,1))*(w.b(j) - w.A(j,2)*dynamics.G(2));
                    if(abs(pd)==Inf)
                        s(j) = Inf;
                    else
                        s(j) = abs(pd - dynamics.G(1));
                    end
                    % Find the correct intersection of the boundary ray to the line
                end
            end
            s = min(s);
            w_approx_sphere = WrenchSetSphere(G,s);
        end
    end
end

