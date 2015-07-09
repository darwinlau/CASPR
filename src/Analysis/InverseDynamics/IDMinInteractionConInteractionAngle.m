classdef IDMinInteractionConInteractionAngle < IDFunction
    %IDQUADFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        weights
        angles
    end
    
    methods
        function q = IDMinInteractionConInteractionAngle(weights, angles)
            % 6p x 1 vector of weights (weight for each component of each
            % joint)
            q.weights = weights;
            q.angles = angles;
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
            
            Q = cell(1,dynamics.numLinks);
            Q(:) = {zeros(dynamics.numCables, dynamics.numCables)};
            l = cell(1,dynamics.numLinks);
            l(:) = {zeros(dynamics.numLinks, 1)};
            r = cell(1,dynamics.numLinks);
            r(:) = {0};
            
            for k = 1:dynamics.numLinks
                mu_a = tan(obj.angles(k))^2;
                a_x = a(6*k-5);
                a_y = a(6*k-4);
                a_z = a(6*k-3);
                H_x_vector = F_T(6*k-5, :);
                H_y_vector = F_T(6*k-4, :);
                H_z_vector = F_T(6*k-3, :);
                
                Q{k} = (H_x_vector')*H_x_vector + (H_y_vector')*H_y_vector - mu_a*(H_z_vector')*H_z_vector;
                l{k} = (2*a_x*H_x_vector + 2*a_y*H_y_vector - mu_a*2*a_z*H_z_vector)';
                r{k} = mu_a*a_z^2 - a_y^2 - a_x^2;
            end
            
            % Solvers for optitoolbox QCQP problems : CLP (seems to not work), OOQP, SCIP, MATLAB
            opts = optiset('solver', 'IPOPT');
            optisolver = opti('qp', H, f, 'eq', A, b, 'bounds', fmin, fmax, 'qc', Q, l, r, 'options', opts);
            
            [dynamics.cableForces, Qf, exitflag, id_info] = solve(optisolver, obj.f_previous);
            
            id_info
            
            if (exitflag ~= 1)
                dynamics.cableForces = dynamics.cableDynamics.forcesInvalid;
                Qf = inf;
                id_exit_type = IDFunction.DisplayOptiToolboxError(exitflag);
            end
            obj.f_previous = dynamics.cableForces;
        end
    end 
end