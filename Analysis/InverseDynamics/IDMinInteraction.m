classdef IDMinInteraction < IDFunction
    %IDQUADFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        weights
    end
    
    methods
        function q = IDMinInteraction(weights)
            % 6p x 1 vector of weights (weight for each component of each
            % joint)
            q.weights = weights;
        end
        
        function [Qf, id_exit_type, id_info] = resolve(obj, dynamics)
            id_exit_type = IDExitType.NO_ERROR;
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A, b] = IDFunction.GetEoMConstraints(dynamics);        
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;
            
            H1 = zeros(1, dynamics.numCables);
            H2 = zeros(dynamics.numCables, dynamics.numCables);
            a = dynamics.P'*(dynamics.bodyDynamics.M_b*dynamics.q_ddot + dynamics.bodyDynamics.C_b - dynamics.bodyDynamics.G_b);
            F_T = dynamics.P'*dynamics.V';
            
            for k = 1:dynamics.numLinks
                for dof = 1:6
                    ax = a(6*(k-1)+dof);
                    H_vector = F_T(6*(k-1)+dof, :);
                    H1 = H1 + obj.weights(6*(k-1)+dof)*2*ax*H_vector;
                    H2 = H2 + obj.weights(6*(k-1)+dof)*(H_vector')*H_vector;
                end
            end
            H = 2*H2;
            f = H1';
            
            % Solvers for optitoolbox QP problems : CLP (seems to not work), OOQP, SCIP, MATLAB
            opts = optiset('solver', 'IPOPT');
            optisolver = opti('qp', H, f, 'eq', A, b, 'bounds', fmin, fmax, 'options', opts);
            [dynamics.cableForces, Qf, exitflag, id_info] = solve(optisolver, obj.f_previous);
           
            id_info
            
            if (exitflag ~= 1)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Qf = inf;
                id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            end
            obj.f_previous = dynamics.cableForces;
                       
%             [forces, Qf, exitflag] = quadprog(H, f, [], [], A, b, fmin, fmax, [], optimset('Display', 'off', 'Algorithm', 'interior-point-convex'));
%             
%             if (exitflag ~= 1)
%                 [forces, Qf, exitflag] = quadprog(H, f, [], [], A, b, fmin, fmax, [], optimset('Display', 'off', 'Algorithm', 'active-set'));
%             end
%             if (exitflag ~= 1)
%                 forces = [];
%                 Qf = -1;
%                 switch exitflag
%                     case -2
%                         fprintf('Infeasible\n');
%                         id_exit_type = InvDynExitType.INFEASIBLE;
%                     otherwise
%                         fprintf('Quadprog solver error : Code %d\n', exitflag);
%                         id_exit_type = InvDynExitType.SOLVER_ERROR;
%                 end
%             end
        end
    end
    
end

