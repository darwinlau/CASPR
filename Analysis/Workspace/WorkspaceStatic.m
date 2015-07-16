classdef WorkspaceStatic < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        function id = WorkspaceStatic()
            id.type = WorkspaceType.SW;
        end
        
        function inWorkspace = evaluate(obj,dynamics)
           % Test if the pose can be an equilibrium pose
           H       =   eye(dynamics.numCables+1);
           f       =   zeros(dynamics.numCables+1,1);
           A       =   [];
           b       =   [];
           Aeq     =   [dynamics.M\dynamics.L',dynamics.M\dynamics.G];
           Aeq     =   (abs(Aeq)>1e-6).*Aeq;
           beq     =   zeros(dynamics.numDofs,1);
           lb      =   [1e-6*ones(dynamics.numCables+1,1)];
           ub      =   [Inf*ones(dynamics.numCables+1,1)];
           options =   optimset('display','off');
           [~,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
           if(exit_flag == 1)
               inWorkspace = 1;
           else
               inWorkspace = 0;
           end
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % This file may need a dynamics object added at a later date
            tol = 1e-6;
            isConnected = sum((abs(workspace(:,i) - workspace(:,j)) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

% Is it An equilibirum
% If so Linearise
% Check positive controllability