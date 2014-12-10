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
            Aeq     =   -L';
            beq     =   zeros(dynamics.numDofs,1);
            lb      =   1e-9*ones(dynamics.numCables,1);
            ub      =   Inf*ones(dynamics.numCables,1);
            [~,~,exit_flag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],obj.options);
            % Test if the Jacobian is full rank
            inWorkspace = (exit_flag==1) && (L_rank == dynamics.numDofs);
            % Calculate semi-singular value
            if(inWorkspace)
                semi_singular = obj.find_semi_singular_value(dynamics);
            else
                semi_singular = 0;
                disp('no no no')
            end
        end
        
        %% Determine the semi_singular value using the test given from semi_singular type
        function semi_singular_value = find_semi_singular_value(obj,dynamics)
            
            % Semi_singular_type = 0 - inner product based approach
            %                      1 - Euclidean norm based approach                    
            if(obj.semi_singular_type == 0)
                % Initialise necessary parameters
                Gram                =   zeros(dynamics.numCables); % This matrix could also be used for separating hyperplane test
                semi_singular_value =   1;
                L                   =   dynamics.L;
                % Determine candidate semi_singular values
                % THIS SHOULD BE CHANGED FOR THE GENERAL CASE (IE FOR ALL
                % POSSIBLE HYPERPLANES)
                for i=1:dynamics.numCables
                    % Compute the Gram matrix
                    perp = [L(i,2);-L(i,1)];
                    for j = i:dynamics.numCables
                        Gram(i,j) = L(j,:)*perp/(norm(L(j,:))*norm(perp));
                        Gram(j,i) = -Gram(i,j);
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
            elseif(obj.semi_singular_type == 1)
                % Initialise necessary parameters
                Gram                =   zeros(dynamics.numCables); % This matrix could also be used for separating hyperplane test
                Dist                =   zeros(dynamics.numCables);
                semi_singular_value =   1e50;
                L                   = dynamics.L;
                % Determine gram and distance matrix values
                % THIS SHOULD BE CHANGED FOR THE GENERAL CASE
                for i=1:dynamics.numCables
                    % Compute the Gram and Distance Matrices
                    perp = [L(i,2);-L(i,1)];
                    for j = i:dynamics.numCables
                        Gram(i,j) = L(j,:)*perp/(norm(L(j,:))*norm(perp));
                        Gram(j,i) = -Gram(i,j);
                        n = (L(i,1)*L(j,1)+L(i,2)*L(j,2))/(L(i,1)^2+L(i,2)^2);
                        Dist(i,j) = norm(L(j,:)-n*L(i,:));
                        Dist(j,i) = Dist(i,j);
                    end
                    % Determine the necessary distance for each side of the
                    % hyperplane
                    temp_pos = max((Dist(i,:).*(Gram(i,:)>0)).^2);
                    temp_neg = max((Dist(i,:).*(Gram(i,:)<0)).^2);
                    temp = min([temp_pos,temp_neg]);
                    if(temp<semi_singular_value)
                        semi_singular_value = temp;
                    end
                end
            end
            
            
            % Detect the singular values (to check if close to a loss
            % of rank
            S = svd(L'*L);
            singular_value = min(S);
            % Display the singular and semi-singular values for the system
            str =sprintf('SV: %f, SSV: %f',singular_value,semi_singular_value);
            disp(str);
            semi_singular_value = 20*semi_singular_value+1;
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6;
            isConnected = sum((abs(workspace(:,i) - workspace(:,j)) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

