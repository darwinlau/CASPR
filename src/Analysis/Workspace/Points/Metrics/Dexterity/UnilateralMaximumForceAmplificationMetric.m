% An approximation of the dexterity measure which applies to 
% systems subject to unilateral constraints.
%
% Please cite the following paper when using this algorithm:
% R. Kurtz and V. Hayward, "Dexterity measures with unilateral actuation 
% constraints: the n+ 1 case", Advanced Robotics, vol. 9, no. 5, pp.
% 561-577, 1994.
%
% This method has been modified to consider arbitrary cable numbers.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : 
classdef UnilateralMaximumForceAmplificationMetric < WorkspaceMetricBase
    % Constants that needs to be defined from parent
    properties (Constant)
        type = WorkspaceMetricType.UNILATERAL_MAXIMUM_FORCE_AMPLIFICATION;
        metricMin = 0;
        metricMax = Inf;
    end
    
    properties (Access = protected)
        options = [];               % Solver options for QP
    end
    
    methods
        % Constructor
        function m = UnilateralMaximumForceAmplificationMetric()
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(obj, dynamics)
            % Determine the Jacobian Matrix
            L = dynamics.L_active;
            % Compute singular values of jacobian matrix
            Sigma = svd(-L');
            % Compute the condition number
            k = 1/min(Sigma);
            % For the moment calculate the UMFA assuming that all cables are
            % used
            H = eye(dynamics.numCablesActive);
            f = zeros(dynamics.numCablesActive,1);
            Aeq = -L';
            beq = zeros(dynamics.numDofs,1);
            lb = ones(dynamics.numCablesActive,1);
            ub = Inf*ones(dynamics.numCablesActive,1);
            [u,~,exit_flag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],obj.options);
            if((exit_flag == 1) && (rank(L) == dynamics.numDofs))
                % ADD A LATER FLAG THAT CAN USE WCW IF ALREADY TESTED
                h = u/norm(u);
                h_min = min(h);
                v = sqrt(dynamics.numCablesActive+1)*(1/k)*(h_min/sqrt(h_min^2+1));
                % Now check if a cable is not necessary (this is only one
                % cable for now future work will look at combinatorics)
                for i=1:dynamics.numCablesActive
                    W = eye(dynamics.numCablesActive);
                    W(i,:) = [];
                    % Determine necessary variables for test
                    L_m       =   W*L; % Cable Jacobian
                    L_rank  =   rank(L');   % Cable Jacobian Rank
                    % Test if Jacobian has a positive spanning subspace
                    H       =   eye(dynamics.numCablesActive-1);
                    f       =   zeros(dynamics.numCablesActive-1,1);
                    Aeq     =   -L_m';
                    lb      =   ones(dynamics.numCablesActive-1,1);
                    ub      =   Inf*ones(dynamics.numCablesActive-1,1);
                    [u,~,exit_flag] = quadprog(H,f,[],[],Aeq,beq,lb,ub,[],obj.options);
                    % Test if the Jacobian is full rank
                    if((exit_flag==1) && (L_rank == dynamics.numDofs))
                        h = u/norm(u);
                        Sigma = svd(-L_m');
                        k = 1/min(Sigma);
                        h_min = min(h);
                        temp_v = sqrt(dynamics.numCablesActive)*(1/k)*(h_min/(sqrt(h_min^2+1)));
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