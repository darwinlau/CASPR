classdef TensionFactorMetric < Metric
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        %% Constructor
        function m = TensionFactorMetric()
            m.options   =    	optimset('display','off');
        end
        
        %% Evaluate Functions
        function v = evaluate(obj,dynamics)
            % Determine the Jacobian Matrix
            L = dynamics.L;
            % Compute singular values of jacobian matrix
            Sigma = svd(-L');
            % Compute the condition number
            k = max(Sigma)/min(Sigma);
            % For the moment calculate the UD assuming that all cables are
            % used
            [u,~,exit_flag] = linprog(ones(1,dynamics.numCables),[],[],-L',zeros(dynamics.numDofs,1),1e-6*ones(dynamics.numCables,1),1e6*ones(dynamics.numCables,1),[],obj.options);
            if((exit_flag == 1) && (rank(L) == dynamics.numDofs))
                % ADD A LATER FLAG THAT CAN USE WCW IF ALREADY TESTED
                h = u/norm(u);
                h_min = min(h); h_max = max(h);
                v = (1/k)*(h_min/h_max);
                % Now check if a cable is not necessary (this is only one
                % cable for now future work will look at combinatorics)
                for i=1:dynamics.numCables
                    W = eye(dynamics.numCables);
                    W(i,:) = [];
                    % Determine necessary variables for test
                    L_m       =   W*L; % Cable Jacobian
                    L_rank  =   rank(L');   % Cable Jacobian Rank
                    % Test if Jacobian has a positive spanning subspace
                    f       =   ones(dynamics.numCables-1,1);
                    Aeq     =   -L_m';
                    lb      =   1e-9*ones(dynamics.numCables-1,1);
                    ub      =   Inf*ones(dynamics.numCables-1,1);
                    [u,~,exit_flag] = linprog(f,[],[],Aeq,zeros(dynamics.numDofs,1),lb,ub,[],obj.options);
                    % Test if the Jacobian is full rank
                    if((exit_flag==1) && (L_rank == dynamics.numDofs))
                        h = u/norm(u);
                        Sigma = svd(-L_m');
                        k = max(Sigma)/min(Sigma);
                        h_min = min(h); h_max = max(h);
                        temp_v = (1/k)*(h_min/h_max);
                        if(temp_v > v)
                            v = temp_v;
                        end
                    end
                end
            else
                v = 0;
            end
        end
        
        function v = workspaceCheck(obj,type)
            if(type == WorkspaceType.WCW)
                v = 1;
            else
                v = 0;
            end
        end
    end
end