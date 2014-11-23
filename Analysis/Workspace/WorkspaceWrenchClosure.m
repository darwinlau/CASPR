classdef WorkspaceWrenchClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        function id = WorkspaceWrenchClosure()
        end
        
        function [inWorkspace,max_vel] = evaluate(obj,dynamics)
            % Evalute wrench closure condition, return true if it is
            % satisfied.
            L       =   dynamics.L;
            L_rank  =   rank(L');
            H       =   eye(dynamics.numCables);
            f       =   zeros(dynamics.numCables,1);
            A       =   [];
            b       =   [];
            Aeq     =   -L';
            beq     =   zeros(dynamics.numDofs,1);
            lb      =   1e-9*ones(dynamics.numCables,1);
            ub      =   Inf*ones(dynamics.numCables,1);
            options =   optimset('display','off');
            [~,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
            inWorkspace = (exit_flag==1) && (L_rank == dynamics.numDofs);
            max_vel = Inf;
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6;
            isConnected = sum((abs(workspace(:,i) - workspace(:,j)) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

