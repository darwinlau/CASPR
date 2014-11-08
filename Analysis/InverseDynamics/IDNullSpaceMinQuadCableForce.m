classdef IDNullSpaceMinQuadCableForce < IDFunction
    %IDQUADFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    methods        
        function id = IDNullSpaceMinQuadCableForce(num_cables)
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
%             delta_g = zeros(dynamics.numCables, 1);
%             for i = 1:dynamics.NumCables
%                 delta_g(i) = -0.5*obj.f_previous(i);
%             end
                                 
            delta_g = -0.5*obj.f_previous;
            
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
                id_exit_type = DisplayOptiToolboxError(exitflag);
            else
                forces = c + A_null*v;
                dynamics.cableForces = forces;
            end
            obj.f_previous = dynamics.cableForces;
        end
    end
    
end

