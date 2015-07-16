classdef WorkspaceWrenchClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        options                         % The options for the wrench closure
    end
    
    methods
        %% Constructor for wrench closure workspace
        function w = WorkspaceWrenchClosure()
            w.options               =   optimset('display','off');
            w.type                  =   WorkspaceType.WCW;
        end
        
        %% Evaluate the wrench closure condition return true if satisfied 
        function inWorkspace = evaluate(obj,dynamics)
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
            % Test if the Jacobian is full rank
            inWorkspace = (exit_flag==1) && (L_rank == dynamics.numDofs);
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6; l_x = size(workspace,1) - 1;
            isConnected = sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(1:l_x,i) - workspace(1:l_x,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

