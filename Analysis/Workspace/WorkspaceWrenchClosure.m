classdef WorkspaceWrenchClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        semi_singular_type              % The method for determining semisingular values
        options                         % The options for the wrench closure
    end
    
    methods
        %% Constructor for wrench closure workspace
        function w = WorkspaceWrenchClosure(semi_singular_type)
            w.semi_singular_type    = semi_singular_type;
            w.options               =   optimset('display','off');
        end
        
        %% Evaluate the wrench closure condition return true if satisfied 
        function [inWorkspace,semi_singular] = evaluate(obj,dynamics)
            % -------------------------------------------------------------
            % This function tests if the cable joint jacobian positively
            % spans the space.
            % -------------------------------------------------------------
            
            % Determine necessary variables for test             
            L       =   dynamics.L; % Cable Jacobian
            L_rank  =   rank(L');   % Cable Jacobian Rank
            % Test if Jacobian is a positive spanning subspace
            H       =   eye(dynamics.numCables);
            f       =   zeros(dynamics.numCables,1);
            Aeq     =   -dynamics.M\L';
            beq     =   zeros(dynamics.numDofs,1);
            lb      =   1e-6*ones(dynamics.numCables,1);
            ub      =   Inf*ones(dynamics.numCables,1);
            [u,~,exit_flag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],obj.options);
%             [u,~,exit_flag] = linprog(ones(dynamics.numCables,1),[],[],Aeq,beq,1000*lb,ub,[],obj.options);
            % Test if the Jacobian is full rank
            inWorkspace = (exit_flag==1) && (L_rank == dynamics.numDofs);
            semi_singular = 0;
%             if(norm(dynamics.q - [pi/6;pi/3])<0.01)
%                 dynamics.M
%                 dynamics.C
%                 dynamics.L
%                 -dynamics.M\L'
%                 -[1.5,1]*(dynamics.M\L')
%                 [2,1]*(dynamics.M\L')
%                 sfdjhdjh
%             end
%             figure;
% %             axis([-7e-3,7e-3,-7e-3,7e-3])
%             hold on;
%             grid on;
%             for i=1:size(Aeq,2)
% %             for i=3
%                 plot([0,Aeq(1,i)],[0,Aeq(2,i)])
%             end
%             if((abs(dynamics.q(1) - pi/2)<pi/18)&&(abs(dynamics.q(2) + 2*pi/9)<pi/18))
%                 dynamics.q
%                 L
%             end
            % Calculate semi-singular value
%             if(inWorkspace)
% %                 nd = sqrt(max(diag(L*L')));                
% % %                 nd = 1;
% %                 max_i = 100;
% %                 for i = 1:max_i
% %                     [~,v,~] = quadprog(H,f,[],[],Aeq,nd*[cos(i*pi/(max_i/2));sin(i*pi/(max_i/2))],lb,ub,[],obj.options);
% %                     semi_singular = sqrt(v)/max_i + semi_singular;
% %                 end
% %                 semi_singular = 100/semi_singular + 5;
%                 if(obj.semi_singular_type == -1)
%                     semi_singular = 1;
%                 else
%                     semi_singular = obj.find_semi_singular_value(dynamics,u);
%                 end
% %                 semi_singular = 1;
%             else
%                 semi_singular = 0;
% %                 semi_singular = obj.find_neg_semi_singular_value(dynamics);
% %                 disp('no no no')
%             end
        end
        
        %% Determine the semi_singular value using the test given from semi_singular type
        function semi_singular_value = find_semi_singular_value(obj,dynamics,opt_iso_vec)
            
            % Semi_singular_type = 0 - inner product based approach
            %                      1 - Euclidean norm based approach                    
            if(obj.semi_singular_type == 0)
                % Initialise necessary parameters
                numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                semi_singular_value =   1;
                L                   =   dynamics.L;
                % Determine candidate semi_singular values
                
                if(dynamics.numDofs == 2)
                elseif(dynamics.numDofs == 3)
                    hyperplane_index = zeros(dynamics.numDofs-1,numPlanes);
                    k = 1;
                    for i = 1:dynamics.numCables
                        for j = i+1:dynamics.numCables
                            hyperplane_index(:,k) = [i;j];
                            k = k+1;
                        end
                    end
                end
                for i=1:numPlanes
                    % Compute the Gram matrix
                    if(dynamics.numDofs == 2)
                        perp = [L(i,2);-L(i,1)];
                        no_test_list = i;
                    elseif(dynamics.numDofs == 3)
                        perp = cross(L(hyperplane_index(1,i),:),L(hyperplane_index(2,i),:))';
                        no_test_list = hyperplane_index(:,i);
                    end
                    for j = 1:dynamics.numCables
                        if(sum(j == no_test_list)>0)
                        else
                            Gram(i,j) = L(j,:)*perp/(norm(L(j,:))*norm(perp));
                            %                             Gram(j,i) = -Gram(i,j);
                        end
                    end
                    % Determine the necessary angle for each side of the
                    % hyperplane
                    temp_pos = max((Gram(i,:).*(Gram(i,:)>0)).^2);
                    temp_neg = max((Gram(i,:).*(Gram(i,:)<0)).^2);
                    temp = min([temp_pos,temp_neg]);
                    % Update semi_singular value if smaller than previous
                    if(temp<semi_singular_value)
                        semi_singular_value = temp;
                    end
                end
                Sigma = svd(L);
                k = min(Sigma)/max(Sigma);
                semi_singular_value = k*sqrt(semi_singular_value);
            elseif(obj.semi_singular_type == 1)
                 % Initialise necessary parameters
                numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                semi_singular_value =   1;
                L                   =   dynamics.L;
                % Determine candidate semi_singular values
                
                if(dynamics.numDofs == 2)
                elseif(dynamics.numDofs == 3)
                    hyperplane_index = zeros(dynamics.numDofs-1,numPlanes);
                    k = 1;
                    for i = 1:dynamics.numCables
                        for j = i+1:dynamics.numCables
                            hyperplane_index(:,k) = [i;j];
                            k = k+1;
                        end
                    end
                end
                for i=1:numPlanes
                    % Compute the Gram matrix
                    if(dynamics.numDofs == 2)
                        perp = [L(i,2);-L(i,1)];
                        no_test_list = i;
                    elseif(dynamics.numDofs == 3)
                        perp = cross(L(hyperplane_index(1,i),:),L(hyperplane_index(2,i),:))';
                        no_test_list = hyperplane_index(:,i);
                    end
                    for j = 1:dynamics.numCables
                        if(sum(j == no_test_list)>0)
                        else
                            Gram(i,j) = L(j,:)*perp/(norm(perp));
                            %                             Gram(j,i) = -Gram(i,j);
                        end
                    end
                    % Determine the necessary angle for each side of the
                    % hyperplane
                    temp_pos = max((Gram(i,:).*(Gram(i,:)>0)).^2);
                    temp_neg = max((Gram(i,:).*(Gram(i,:)<0)).^2);
                    temp = min([temp_pos,temp_neg]);
                    % Update semi_singular value if smaller than previous
                    if(temp<semi_singular_value)
                        semi_singular_value = temp;
                    end
                end
            elseif(obj.semi_singular_type == 2)
                % Initialise necessary parameters
                numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                Dist                =   zeros(numPlanes,dynamics.numCables);
                semi_singular_value =   1;
                L                   = dynamics.L;
                
                % Determine gram and distance matrix values
                % THIS SHOULD BE CHANGED FOR THE GENERAL CASE
                if(dynamics.numDofs == 2)
                elseif(dynamics.numDofs == 3)
                    hyperplane_index = zeros(dynamics.numDofs-1,numPlanes);
                    k = 1;
                    for i = 1:dynamics.numCables
                        for j = i+1:dynamics.numCables
                            hyperplane_index(:,k) = [i;j];
                            k = k+1;
                        end
                    end
                end
                for i=1:numPlanes
                    % Compute the Gram and Distance Matrices
                    if(dynamics.numDofs == 2)
                        perp = [L(i,2);-L(i,1)];
                        no_test_list = i;
                    elseif(dynamics.numDofs == 3)
                        perp = cross(L(hyperplane_index(1,i),:),L(hyperplane_index(2,i),:))';
                        no_test_list = hyperplane_index(:,i);
                    end
                    for j = 1:dynamics.numCables
                        if(sum(j == no_test_list)>0)
                        else
                            Gram(i,j) = L(j,:)*perp/(norm(L(j,:))*norm(perp));
                            %                         Gram(j,i) = -Gram(i,j);
                            if(dynamics.numDofs == 2)
                                n   = (L(i,1)*L(j,1)+L(i,2)*L(j,2))/(L(i,1)^2+L(i,2)^2);
                                m0  =   n*L(i,:);
                            elseif(dynamics.numDofs == 3)
                                k1  =   hyperplane_index(1,i);
                                k2  =   hyperplane_index(2,i);
                                s   =   (L(k1,1)*L(k2,2)^2*L(j,1) + L(k1,1)*L(k2,3)^2*L(j,1) + L(k1,2)*L(k2,1)^2*L(j,2) + L(k1,2)*L(k2,3)^2*L(j,2) + L(k1,3)*L(k2,1)^2*L(j,3) + L(k1,3)*L(k2,2)^2*L(j,3) - L(k1,1)*L(k2,1)*L(k2,2)*L(j,2) - L(k1,2)*L(k2,1)*L(k2,2)*L(j,1) - L(k1,1)*L(k2,1)*L(k2,3)*L(j,3) - L(k1,3)*L(k2,1)*L(k2,3)*L(j,1) - L(k1,2)*L(k2,2)*L(k2,3)*L(j,3) - L(k1,3)*L(k2,2)*L(k2,3)*L(j,2))/(L(k1,1)^2*L(k2,2)^2 + L(k1,1)^2*L(k2,3)^2 - 2*L(k1,1)*L(k1,2)*L(k2,1)*L(k2,2) - 2*L(k1,1)*L(k1,3)*L(k2,1)*L(k2,3) + L(k1,2)^2*L(k2,1)^2 + L(k1,2)^2*L(k2,3)^2 - 2*L(k1,2)*L(k1,3)*L(k2,2)*L(k2,3) + L(k1,3)^2*L(k2,1)^2 + L(k1,3)^2*L(k2,2)^2);
                                t   =   (L(k1,2)^2*L(k2,1)*L(j,1) + L(k1,1)^2*L(k2,2)*L(j,2) + L(k1,3)^2*L(k2,1)*L(j,1) + L(k1,1)^2*L(k2,3)*L(j,3) + L(k1,3)^2*L(k2,2)*L(j,2) + L(k1,2)^2*L(k2,3)*L(j,3) - L(k1,1)*L(k1,2)*L(k2,1)*L(j,2) - L(k1,1)*L(k1,2)*L(k2,2)*L(j,1) - L(k1,1)*L(k1,3)*L(k2,1)*L(j,3) - L(k1,1)*L(k1,3)*L(k2,3)*L(j,1) - L(k1,2)*L(k1,3)*L(k2,2)*L(j,3) - L(k1,2)*L(k1,3)*L(k2,3)*L(j,2))/(L(k1,1)^2*L(k2,2)^2 + L(k1,1)^2*L(k2,3)^2 - 2*L(k1,1)*L(k1,2)*L(k2,1)*L(k2,2) - 2*L(k1,1)*L(k1,3)*L(k2,1)*L(k2,3) + L(k1,2)^2*L(k2,1)^2 + L(k1,2)^2*L(k2,3)^2 - 2*L(k1,2)*L(k1,3)*L(k2,2)*L(k2,3) + L(k1,3)^2*L(k2,1)^2 + L(k1,3)^2*L(k2,2)^2);
                                m0  =   s*L(k1,:) + t*L(k2,:);
                            end
                            Dist(i,j) = norm(L(j,:)-m0);
                            %                         Dist(j,i) = Dist(i,j);
                        end
                    end
                    % Determine the necessary distance for each side of the
                    % hyperplane
                    temp_pos = max((Dist(i,:).*(Gram(i,:)>0)).^2);
                    temp_neg = max((Dist(i,:).*(Gram(i,:)<0)).^2);
                    temp = min([temp_pos,temp_neg]);
                    semi_singular_value = semi_singular_value*temp;
                end
                semi_singular_value = nthroot(semi_singular_value,4);
            elseif(obj.semi_singular_type == 3)
                    % Initialise necessary parameters
                    numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                    Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                    semi_singular_value =   1;
                    L                   =   dynamics.L;
                    % Determine candidate semi_singular values
                    
                    if(dynamics.numDofs == 2)
                    elseif(dynamics.numDofs == 3)
                        hyperplane_index = zeros(dynamics.numDofs-1,numPlanes);
                        k = 1;
                        for i = 1:dynamics.numCables
                            for j = i+1:dynamics.numCables
                                hyperplane_index(:,k) = [i;j];
                                k = k+1;
                            end
                        end
                    end
                    for i=1:numPlanes
                        % Compute the Gram matrix
                        if(dynamics.numDofs == 2)
                            perp = [L(i,2);-L(i,1)];
                            no_test_list = i;
                        elseif(dynamics.numDofs == 3)
                            perp = cross(L(hyperplane_index(1,i),:),L(hyperplane_index(2,i),:))';
                            no_test_list = hyperplane_index(:,i);
                        end
                        for j = 1:dynamics.numCables
                            if(sum(j == no_test_list)>0)
                            else
                                Gram(i,j) = L(j,:)*perp/(norm(L(j,:))*norm(perp));
                                %                             Gram(j,i) = -Gram(i,j);
                            end
                        end
                        % Determine the necessary angle for each side of the
                        % hyperplane
                        temp_pos = max((Gram(i,:).*(Gram(i,:)>0)).^2);
                        temp_neg = max((Gram(i,:).*(Gram(i,:)<0)).^2);
                        temp = min([temp_pos,temp_neg]);
                        semi_singular_value = temp*semi_singular_value;
                    end
                    Sigma = svd(L'*L);
                    k = min(Sigma)/max(Sigma);
                    semi_singular_value = k*nthroot(semi_singular_value,dynamics.numCables);
            elseif(obj.semi_singular_type == 4)
                L = dynamics.L;
                h = opt_iso_vec/norm(opt_iso_vec);
                Sigma = svd(-L');
                h_min = min(h);
                k = max(Sigma)/min(Sigma);
                if(h_min < 0)
                    h_min = min(-h);
                    semi_singular_value = sqrt(dynamics.numCables+1)*(1/k)*(h_min/(sqrt(h_min^2+1)));
                else
                    h_min = min(h);
                    semi_singular_value = sqrt(dynamics.numCables+1)*(1/k)*(h_min/(sqrt(h_min^2+1)));
                end
                for i=1:dynamics.numCables
                    W = eye(dynamics.numCables);
                    W(i,:) = [];
                    % Determine necessary variables for test
                    L_m       =   W*L; % Cable Jacobian
                    L_rank  =   rank(L');   % Cable Jacobian Rank
                    % Test if Jacobian is a positive spanning subspace
                    H       =   eye(dynamics.numCables-1);
                    f       =   zeros(dynamics.numCables-1,1);
                    Aeq     =   -L_m';
                    %             q1 = dynamics.q(1);
                    %             q2 = dynamics.q(2);
                    %             Aeq = [1,cos(q1),cos(q2);0,sin(q1),sin(q2)];
                    %             L_rank = rank(Aeq);
                    beq     =   zeros(dynamics.numDofs,1);
                    lb      =   1e-9*ones(dynamics.numCables-1,1);
                    ub      =   Inf*ones(dynamics.numCables-1,1);
                    [u,~,exit_flag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],obj.options);
                    %             [u,~,exit_flag] = linprog(ones(dynamics.numCables,1),[],[],Aeq,beq,1000*lb,ub,[],obj.options);
                    % Test if the Jacobian is full rank
                    if((exit_flag==1) && (L_rank == dynamics.numDofs))
                        h = u/norm(u);
                        Sigma = svd(-L_m');
                        k = max(Sigma)/min(Sigma);
                        if(sum(h<0)>0)
                            if(sum(h <0) == dynamics.numCables)
                                h_min = min(-h);
                                temp = sqrt(dynamics.numCables+1)*(1/k)*(h_min/(sqrt(h_min^2+1)));
                            else
                                temp = 0;
                            end
                        else
                             h_min = min(h);
                             temp = sqrt(dynamics.numCables+1)*(1/k)*(h_min/(sqrt(h_min^2+1)));
                        end 
                        if(temp>semi_singular_value)
                            semi_singular_value = temp;
                        end
                    end
                end
            elseif(obj.semi_singular_type == 5)
                L = dynamics.L;
                h = opt_iso_vec/norm(opt_iso_vec);
                Sigma = svd(-L');
                k = max(Sigma)/min(Sigma);
                if(sum(h<0)>0)
                    if(sum(h <0) == dynamics.numCables)
                        h_min = min(-h); h_max = max(-h);
%                         semi_singular_value = h_min/h_max;
                        semi_singular_value = (1/k)*h_min/h_max;
                    else
                        semi_singular_value = 0;
                    end
                else
                    h_min = min(h); h_max = max(h);
%                     semi_singular_value = h_min/h_max;
                    semi_singular_value = (1/k)*h_min/h_max;
                end
                %% Extend later to cover larger numbers of redundant cables.
                for i=1:dynamics.numCables
                    W = eye(dynamics.numCables);
                    W(i,:) = [];
                    % Determine necessary variables for test
                    L_m       =   W*L; % Cable Jacobian
                    L_rank  =   rank(L');   % Cable Jacobian Rank
                    % Test if Jacobian is a positive spanning subspace
                    H       =   eye(dynamics.numCables-1);
                    f       =   zeros(dynamics.numCables-1,1);
                    Aeq     =   -L_m';
                    %             q1 = dynamics.q(1);
                    %             q2 = dynamics.q(2);
                    %             Aeq = [1,cos(q1),cos(q2);0,sin(q1),sin(q2)];
                    %             L_rank = rank(Aeq);
                    beq     =   zeros(dynamics.numDofs,1);
                    lb      =   1e-9*ones(dynamics.numCables-1,1);
                    ub      =   Inf*ones(dynamics.numCables-1,1);
                    [u,~,exit_flag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],obj.options);
                    %             [u,~,exit_flag] = linprog(ones(dynamics.numCables,1),[],[],Aeq,beq,1000*lb,ub,[],obj.options);
                    % Test if the Jacobian is full rank
                    if((exit_flag==1) && (L_rank == dynamics.numDofs))
                        h = u/norm(u);
                        Sigma = svd(-L_m');
                        k = max(Sigma)/min(Sigma);
                        if(sum(h<0)>0)
                            if(sum(h <0) == dynamics.numCables)
                                h_min = min(-h); h_max = max(-h);
                                temp = (1/k)*h_min/h_max;
                            else
                                temp = 0;
                            end
                        else
                            h_min = min(h); h_max = max(h);
                            temp = (1/k)*h_min/h_max;
                        end 
                        if(temp>semi_singular_value)
                            semi_singular_value = temp;
                        end
                    end
                end
            elseif(obj.semi_singular_type == 6)
                
            end
            
            
            % Detect the singular values (to check if close to a loss
            % of rank
            S = svd(L'*L);
            singular_value = min(S);
            % Display the singular and semi-singular values for the system
            str =sprintf('SV: %f, SSV: %f',singular_value,semi_singular_value);
            disp(str);
            semi_singular_value = 50*semi_singular_value+1;
        end
        
        %% Determine the semi_singular value using the test given from semi_singular type
%         function semi_singular_value = find_neg_semi_singular_value(obj,dynamics)
%             
%             % Semi_singular_type = 0 - inner product based approach
%             %                      1 - Euclidean norm based approach
%             if(obj.semi_singular_type == 1)
%                 
%             else
%                 semi_singular_value = 0;
%             end
%             
%             
%             % Detect the singular values (to check if close to a loss
%             % of rank
%             S = svd(L'*L);
%             singular_value = min(S);
%             % Display the singular and semi-singular values for the system
%             str =sprintf('SV: %f, SSV: %f',singular_value,semi_singular_value);
%             disp(str);
%             semi_singular_value = 50*semi_singular_value+1;
%         end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6; l_x = size(workspace,1) - 1;
            isConnected = sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

