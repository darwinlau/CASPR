classdef WorkspaceStaticClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        function id = WorkspaceStaticClosure()
        end
        
        function [inWorkspace,max_acc] = evaluate(obj,dynamics)
           % Test if the pose can be an equilibrium pose
           H       =   eye(dynamics.numCables+1);
           f       =   zeros(dynamics.numCables+1,1);
           A       =   [];
           b       =   [];
           Aeq     =   [dynamics.M\dynamics.L',dynamics.M\dynamics.G];
           Lrank   =    rank(Aeq);
           Aeq     =   (abs(Aeq)>1e-6).*Aeq;
           beq     =   zeros(dynamics.numDofs,1);
           lb      =   [1e-6*ones(dynamics.numCables+1,1)];
           ub      =   [Inf*ones(dynamics.numCables+1,1)];
           options =   optimset('display','off');
           [~,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
           [~,~,exit_flag2] = quadprog(H(1:dynamics.numCables,1:dynamics.numCables),f(1:dynamics.numCables),A,b,Aeq(:,1:dynamics.numCables),beq,lb(1:dynamics.numCables),ub(1:dynamics.numCables),[],options);
           if((Lrank == dynamics.numDofs) && (exit_flag == 1))
               inWorkspace = 1;
               % Now determine the maximum acceleration
               if(exit_flag2 == 1)
                   max_acc = Inf;
               else
                   max_acc = Inf;
                   for i=1:dynamics.numCables
                        a = -(Aeq(:,i)'*Aeq(:,i))/(Aeq(:,dynamics.numCables+1)'*Aeq(:,i));
                        temp_acc = norm(a*Aeq(:,i) + Aeq(:,dynamics.numCables+1));
                        if(temp_acc <= max_acc)
                            max_acc = temp_acc;
                        end
                   end
               end
           else
               inWorkspace = 0;
               max_acc = 0;
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