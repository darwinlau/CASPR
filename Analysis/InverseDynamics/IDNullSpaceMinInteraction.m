classdef IDNullSpaceMinInteraction < IDFunction
    %IDQUADFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        function id = IDNullSpaceMinInteraction(num_cables)            
            id.f_previous = zeros(num_cables, 1);
        end
        
        function [Qf, id_exit_type, id_info] = resolve(obj, dynamics)
            id_exit_type = IDExitType.NO_ERROR;
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A, b] = IDFunction.GetEoMConstraints(dynamics);      
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;  
            H = diag(ones(dynamics.numCables,1));
            f = zeros(dynamics.numCables, 1);
            
            A_inv = pinv(A);
            A_null = (eye(dynamics.numCables) - A_inv*A);
            
            
            H1 = zeros(1, dynamics.numCables);
            H2 = zeros(dynamics.numCables, dynamics.numCables);
            a = dynamics.P'*(dynamics.bodyDynamics.M_b*dynamics.q_ddot + dynamics.bodyDynamics.C_b - dynamics.bodyDynamics.G_b);
            F_T = dynamics.P'*dynamics.V';
            
            for k = 1:dynamics.numLinks
                for dof = 1:6
                    ax = a(6*(k-1)+dof);
                    H_vector = F_T(6*(k-1)+dof, :);
                    H1 = H1 + 2*ax*H_vector;
                    H2 = H2 + (H_vector')*H_vector;
                end
            end
            H_m = 2*H2;
            f_m = H1';       
            %delta_g = zeros(dynamics.NumCables, 1);
            delta_g = -0.7*(H_m*obj.f_previous + f_m);
%             for i = 1:dynamics.NumCables
%                 g_i = 0;
%                 delta_g(i) = -0.1*(dynamics.CableForces(i) - 0);
%             end
            
            A_ineq = [-A_null; A_null];
            c = [A_inv*b + A_null*delta_g];
            b_ineq = [-(fmin - c); fmax - c];
                        
            % Solvers for optitoolbox QP problems : CLP (seems to not work), OOQP, SCIP, MATLAB
            opts = optiset('solver', 'OOQP');
            optisolver = opti('qp', H, f, 'ineq', A_ineq, b_ineq, 'options', opts);
            [v, Qf, exitflag, id_info] = solve(optisolver, obj.f_previous);
            
            id_info
            
            if (exitflag ~= 1)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Qf = inf;
                id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            else
                forces = c + A_null*v;
                dynamics.cableForces = forces;  
            end
        end
    end
    
end

