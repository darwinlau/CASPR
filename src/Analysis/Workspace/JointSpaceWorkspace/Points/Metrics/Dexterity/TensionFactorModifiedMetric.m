% A measure of the relative tension distribution for the cables which can
% be used to evaluate the quality of wrench closure at a specific
% configuration.
%
% Please cite the following paper when using this algorithm:
% C.B Pham, S.H Yeo, G. Yang and I. Chen, "Workspace analysis of fully
% restrained cable-driven manipulators", Robotics and Autonomous Systems, 
% vol. 57, no. 9, pp. 901-912, 2009.
%
% This method has been modified to consider singular value
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : Implementation of the tension factor metric. This now
% includes the singular value and considers the effect of removing cables
classdef TensionFactorModifiedMetric < WorkspaceMetricBase
    % Constants that needs to be defined from parent
    properties (Constant)
        type = WorkspaceMetricType.TENSION_FACTOR_MODIFIED;
        metricMin = 0;
        metricMax = 1;
    end
    
    properties (Access = protected)
        options = [];           % Solver options for LP
    end
    
    methods
        % Constructor
        function m = TensionFactorModifiedMetric()
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(obj, dynamics)
            % Determine the Jacobian Matrix
            L = dynamics.L_active;
            % Compute singular values of jacobian matrix
            Sigma = svd(-L');
            % Compute the condition number
            k = max(Sigma)/min(Sigma);
            % For the moment calculate the UD assuming that all cables are
            % used
            [u,~,exit_flag] = linprog(ones(1,dynamics.numCablesActive),[],[],-L',zeros(dynamics.numDofs,1),1e-6*ones(dynamics.numCablesActive,1),1e6*ones(dynamics.numCablesActive,1),obj.options);
            if((exit_flag == 1) && (rank(L) == dynamics.numDofs))
                % ADD A LATER FLAG THAT CAN USE WCW IF ALREADY TESTED
                h = u/norm(u);
                h_min = min(h); h_max = max(h);
                v = (1/k)*(h_min/h_max);
                % Now check if a cable is not necessary (this is only one
                % cable for now future work will look at combinatorics)
                for i=1:dynamics.numCablesActive
                    W = eye(dynamics.numCablesActive);
                    W(i,:) = [];
                    % Determine necessary variables for test
                    L_m       =   W*L; % Cable Jacobian
                    L_rank  =   rank(L');   % Cable Jacobian Rank
                    % Test if Jacobian has a positive spanning subspace
                    f       =   ones(dynamics.numCablesActive-1,1);
                    Aeq     =   -L_m';
                    lb      =   1e-9*ones(dynamics.numCablesActive-1,1);
                    ub      =   Inf*ones(dynamics.numCablesActive-1,1);
                    [u,~,exit_flag] = linprog(f,[],[],Aeq,zeros(dynamics.numDofs,1),lb,ub,obj.options);
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
    end
end