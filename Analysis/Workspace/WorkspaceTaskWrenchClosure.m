classdef WorkspaceTaskWrenchClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    % THIS IS A WORK IN PROGRESS!!!!!!!!!
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        function id = WorkspaceTaskWrenchClosure()
        end
        
        function [inWorkspace,max_vel] = evaluate(obj,dynamics)
            % Evalute the output closure condition, return true if it is
            % satisfied.
            % THIS CURRENTLY HAS A HARDCODED JACOBIAN.
            % When the necessary Jacobian, lines with comments should be
            % changed to the commented code.
            J = [-sin(dynamics.q(1))-sin(dynamics.q(1)+ dynamics.q(2)),-sin(dynamics.q(1)+ dynamics.q(2))];
            for i = 1:2
                if(abs(J(i))<1e-10)
                    J(i) = 0;
                end
            end 
            JL = -J*inv(dynamics.M)*dynamics.L';  % JL = - dynamics.J*inv(dynamics.M)*dynamics.L';
            JL_rank     =   rank(JL);
%             H           =   eye(dynamics.numCables);
%             f           =   zeros(dynamics.numCables,1);
            A           =   [];
            b           =   [];
            Aeq         =   JL;
            beq         =   zeros(1,1); % beq     =   zeros(dynamics.numOutDofs,1);
            lb          =   1e-6*ones(dynamics.numCables,1);
            ub          =   Inf*ones(dynamics.numCables,1);
            options     =   optimset('display','off');
            [~,~,exit_flag] = linprog(ones(dynamics.numCables,1),A,b,Aeq,beq,lb,ub,[],options);
            [~,~,exit_flag_2] = linprog(ones(dynamics.numCables,1),A,b,dynamics.M\dynamics.L',dynamics.q_ddot_dynamics,zeros(dynamics.numCables,1),ub,[],options);
%             inWorkspace = (exit_flag==1) && (exit_flag_2 == 1) && (JL_rank == 1);
            inWorkspace = (exit_flag==1) && (JL_rank == 1);
            max_vel = 1;
            if(norm(dynamics.q - [pi/6;pi/3])<0.01)
                JL
                dynamics.q
                exit_flag
                JL_rank
            end
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % This file may need a dynamics object added at a later date
            tol = 1e-6;
            L_rank = dynamics.rank(L);
            % Determine the positive span range and null spaces
            q_diff = workspace(:,i) - workspace(:,j);
            % Project q_diff into the null and range spaces
            isConnected = (norm(null_p,2)<tol)&&(norm(range_p,2)<grid.delta_q);
        end
    end
end

