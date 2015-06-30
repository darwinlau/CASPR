classdef TestDriftClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        function id = TestDriftClosure()
        end
        
        function [inWorkspace,max_vel] = evaluate(obj,dynamics)
            % Evalute wrench closure condition, return true if it is
            % satisfied.
            M       =   dynamics.M;
            L       =   dynamics.L;
            L_rank  =   rank(L');
            H2       =   eye(dynamics.numCables+1);
            f2       =   zeros(dynamics.numCables+1,1);
            H       =   eye(dynamics.numCables);
            f       =   zeros(dynamics.numCables,1);
            A       =   [];
            b       =   [];
            Aeq     =   -M\L';
            Aeq2     =   [-M\L',-M\(dynamics.C+dynamics.G)];
           for i = 1:5
               if(norm(Aeq2(:,i))>1e-6)
                Aeq2(:,i) = Aeq2(:,i)/norm(Aeq2(:,i));
               end
           end
            beq     =   zeros(dynamics.numDofs,1);
            lb2      =   1e-6*ones(dynamics.numCables+1,1);
            ub2      =   Inf*ones(dynamics.numCables+1,1);
            lb      =   1e-9*ones(dynamics.numCables,1);
            ub      =   Inf*ones(dynamics.numCables,1);
            options =   optimset('display','off');
            [u1,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
            inWorkspace = (exit_flag==1) && (L_rank == dynamics.numDofs);
            [~,~,exit_flag2] = quadprog(H2,f2,A,b,Aeq2,beq,lb2,ub2,[],options);
            if((inWorkspace==1) && (exit_flag2~=1))
                Aeq
                Aeq2
                for i = 1:5
                   Aeq2(:,i) = Aeq2(:,i)/norm(Aeq2(:,i));
                end
                u1
                pinv(Aeq)*(-M\(dynamics.C+dynamics.G))
                [u1,~,exit_flag] = quadprog(H,f,A,b,Aeq2(:,1:4),beq,lb,ub,[],options);
                exit_flag
                u1
                exit_flag2
%                 [~,~,exit_flag2] = quadprog(H2,f2,A,b,Aeq2,beq,lb2,ub2,[],options);
%                 exit_flag2
                sfdld
            end
%             if(inWorkspace == 1)
%                 [~,~,exit_flag] = quadprog(H(1:dynamics.numCables,1:dynamics.numCables),f(1:dynamics.numCables),A,b,-L',beq,lb(1:dynamics.numCables),ub(1:dynamics.numCables),[],options);
%                 if(exit_flag == 1)
%                     max_vel = 10;
%                     return;
%                 end
%                 % Find the boundary angles
%                 calculated_angles = atan2(Aeq(2,1:3),Aeq(1,1:3));
%                 diff_angle = [abs(calculated_angles(2) - calculated_angles(1));abs(calculated_angles(3) - calculated_angles(1));abs(calculated_angles(2) - calculated_angles(3))];
%                 flag = zeros(3,1);
%                 for j = 1:3
%                     if(diff_angle(j) > pi)
%                         diff_angle(j) = 2*pi-diff_angle(j);
%                         flag(j) = 1;
%                     end
%                 end
%                 [~,i] = max(diff_angle);
%                 if(i == 1)
%                     min_angle = min([calculated_angles(1),calculated_angles(2)]);
%                     max_angle = max([calculated_angles(1),calculated_angles(2)]);
%                 elseif(i == 2)
%                     min_angle = min([calculated_angles(1),calculated_angles(3)]);
%                     max_angle = max([calculated_angles(1),calculated_angles(3)]);
%                 elseif(i == 3)
%                     min_angle = min([calculated_angles(3),calculated_angles(2)]);
%                     max_angle = max([calculated_angles(3),calculated_angles(2)]);
%                 else
%                 end
%                 
%                 if(flag(i) == 1)
%                     for r = 0.5:0.5:10
%                         for t = max_angle:(min_angle+2*pi-max_angle)/10:min_angle+2*pi
%                             qd = [r*cos(2*pi-t);r*sin(2*pi-t)];
%                             dynamics.update(dynamics.q,qd,zeros(size(dynamics.q)));
%                             Aeq     =   [-M\L',-M\(dynamics.C+dynamics.G)];
%                             [~,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
%                             if(exit_flag~=1)
%                                 max_vel = r;
%                                 return;
%                             end
%                         end
%                     end
%                 else
%                     for r = 0.5:0.5:10
%                         for t = min_angle:(max_angle-min_angle)/10:max_angle
%                             qd = [r*cos(2*pi-t);r*sin(2*pi-t)];
%                             dynamics.update(dynamics.q,qd,zeros(size(dynamics.q)));
%                             Aeq     =   [-M\L',-M\(dynamics.C+dynamics.G)];
%                             [~,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
%                             if(exit_flag~=1)
%                                 max_vel = r;
%                                 return;
%                             end
%                         end
%                     end
%                 end
% %             if(inWorkspace==1)
% %                 Aeq
% %                 for r = 1:5
% %                     for t = -pi:pi/10:pi-pi/10
% %                         qd = [r*cos(t);r*sin(t)];
% %                         dynamics.update(dynamics.q,qd,zeros(size(dynamics.q)));
% %                         Aeq     =   [-M\L',-M\(dynamics.C+dynamics.G)];
% %                         Aeq
% %                         [u,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
% %                         exit_flag
% %                         Aeq*u
% %                         if(exit_flag~=1)
% %                             disp('fail')
% %                             r
% %                             max_vel = r;
% %                             if(r<2)
% %                                 asd
% %                             end
% %                             return;
% %                         end
% %                     end
% %                 end
% %             end
%             end
            max_vel = 10;
        end
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6;
            isConnected = sum((abs(workspace(:,i) - workspace(:,j)) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

