% Object representation of the wrench-set
%
% Author        : Darwin LAU
% Created       : 2020
% Description    :
classdef ConvexPolytope < PointsSet
        
    properties (SetAccess = protected)
        numFaces = 0                % Number of faces for the convex polytope
        all_points = []             % Vertices of the convex polytope
        indices = []                % Indices points that represent the verticles of the convex hull
        volume = 0                  % The volume of the polytope
        A = []                      % The polytope is described by A*w <= b
        b = []                      % The polytope is described by A*w <= b
    end
    
    methods        
        % Builds the wrench set using the jacobian and force bounds. This
        % version does not make an external system call.
        % The system model is defined as:
        %   w = As * f + offset
        % Note: As is the structure matrix
        function ws = ConvexPolytope(W)
            ws@PointsSet(W);
            
            W_T = W';
            [K, vol] = convhulln(W_T);
            
            num_faces = size(K,1);
            num_dofs = size(K,2);
            
            t_A = zeros(num_faces, num_dofs);
            t_b = zeros(num_faces, 1);
            
            count = 1;
            
            for i = 1:num_faces
                Wi = W_T(K(i,2:end),:) - repmat(W_T(K(i,1),:),num_dofs-1,1);
                Ti = null(Wi)';
                if(rank(Ti)>1)
                else
                    t_A(count,:) = Ti;
                    t_b(count,:) = t_A(count,:)*W_T(K(i,1),:).';
                    % Find a new vertex not in the face
                    s_flag = 0; j = 1;
                    while(s_flag == 0)
                        if((sum(j==K(i,:))==0)&&(norm(t_A(count,:)*W_T(K(j,1),:)'-t_b(count))>1e-6))
                            s_flag = 1;
                        else
                            j = j+1;
                        end
                    end
                    if(t_A(count,:)*W_T(K(j,1),:)'>t_b(count))
                        t_A(count,:) = -t_A(count,:);
                        t_b(count) = -t_b(count);
                    end
                    count = count + 1;
                end
            end
            
            ws.A = t_A(1:count-1,:);
            ws.b = t_b(1:count-1);
            ws.numDofs = num_dofs;
            ws.numFaces = num_faces;
            ws.all_points = W;
            ws.volume = vol;
            ws.indices = K;
            ws.points = W(:, unique(K));
        end
        
        % Approximate the wrench set with a sphere at position G with
        % radius s. The radius corresponds to the radius of the capacity
        % margin
        function w_approx_sphere = sphereApproximationCapacity(obj,G)
            q = obj.numFaces;
            s = zeros(q,1);
            for j=1:q
                s(j) = (obj.b(j) - obj.A(j,:)*G)/norm(obj.A(j,:));
            end
            s = min(s);
            w_approx_sphere = Hypersphere(G,s);
        end
        
        % Determines the largest sphere enclosed within the wrench set.
        function w_approx_sphere = sphereApproximationChebyshev(obj)
            q = obj.numFaces;
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
            w_approx_sphere = Hypersphere(o(1:2),o(3));
        end
        
        % Approximate the wrench set with the largest sphere containing the
        % reference point.
        function w_approx_sphere = sphereApproximationMax(obj,x_ref,buffer)
            options = optimoptions('fmincon','Algorithm','active-set','Display','off');
            [T,r,~] = fmincon(@(x) costMinRad(x,obj.A,obj.b),[0;0],obj.A,obj.b,[],[],[],[],@(x) constraintPointContained(x,obj.A,obj.b,x_ref,buffer),options);
            w_approx_sphere = Hypersphere(T,r);
        end
        
        % Approximates the wrench set to account for the coriolis.
        function w_approx_sphere = sphereApproximationCoriolis(obj,G,q2)
            % THIS NEEDS TO BE CHANGED LATER
            q = obj.numFaces;
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
            w_approx_sphere = Hypersphere(G,s);
        end
        
        function plotConvexPolytope(obj, ax)
            CASPR_log.Assert(obj.numDofs == 2 || obj.numDofs == 3, 'The ConvexPolytope can only be plotted if it is 2-D or 3-D.')
            if (nargin < 2)
                ax = [];
            end
            
            if (obj.numDofs == 2)
                figure;
                plot(obj.points(1,:), obj.points(2,:), 'k.', 'MarkerSize', 20);
                hold on;
                plot(obj.points(1,obj.indices'), obj.points(2,obj.indices'), 'k', 'LineWidth', 1.5);
                hold off;
                axis equal;
                if (~isempty(ax))
                    axis(ax);
                end
                xlabel('q_1');
                ylabel('q_2');
            elseif (obj.numDofs == 3)
                figure;
                trisurf(obj.indices, obj.points(1,:), obj.points(2,:), obj.points(3,:), 'FaceAlpha', 0.8, 'LineWidth', 1.0, 'FaceColor', [0.7 0.7 0.7]);
                axis equal;
                if (~isempty(ax))
                    axis(ax);
                end
                xlabel('q_1');
                ylabel('q_2');
                zlabel('q_3');
            end
        end
    end
end