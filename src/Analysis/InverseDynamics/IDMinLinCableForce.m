classdef IDMinLinCableForce < IDFunction
    %IDQUADFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        weights
    end
    
    methods
        function q = IDMinLinCableForce(weights)
            q.weights = weights;
        end
        
        function [Qf, id_exit_type, id_info] = resolve(obj, dynamics)
            id_exit_type = IDExitType.NO_ERROR;
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A, b] = IDFunction.GetEoMConstraints(dynamics);     
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;
            
%             if isempty(obj.f_previous)
%                 obj.f_previous = zeros(dynamics.numCables, 1);
%             end
                       
            % Solvers for optitoolbox QP problems : CLP, CSDP, DSDP, GLPK, LIPSOL, LP_SOLVE, OOQP, QSOPT, SCIP
            opts = optiset('solver', 'CLP');
            optisolver = opti('f', obj.weights, 'eq', A, b, 'bounds', fmin, fmax, 'options', opts);
            [dynamics.cableForces, Qf, exitflag, id_info] = solve(optisolver, obj.f_previous);
            
            id_info
            
            if (exitflag ~= 1)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Qf = inf;
                id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            end
            obj.f_previous = dynamics.cableForces;
            
            %[forces, Qf, exitflag] = linprog(obj.Weights, [], [], A, b, fmin, fmax, [], optimset('Display', 'off'));
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

