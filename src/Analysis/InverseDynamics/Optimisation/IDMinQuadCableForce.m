classdef IDMinQuadCableForce < IDSolverFunction
    %IDQUADFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        weights
    end
    
    methods
        function q = IDMinQuadCableForce(weights)
            q.weights = weights;
        end
        
        function [Qf, id_exit_type, id_info] = resolve(obj, dynamics)
            id_exit_type = IDExitType.NO_ERROR;
            H = diag(obj.weights);
            f = zeros(length(obj.weights), 1);
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A, b] = IDFunction.GetEoMConstraints(dynamics);     
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;
            
%             if isempty(obj.f_previous)
%                 obj.f_previous = zeros(dynamics.numCables, 1);
%             end
                       
            % Solvers for optitoolbox QP problems : CLP (seems to not
            % work), OOQP, SCIP, MATLAB, IPOPT
            opts = optiset('solver', 'IPOPT', 'maxiter', 100);
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

