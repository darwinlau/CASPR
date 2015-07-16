classdef IDMinQuadCableForcesConInteractionAngle < IDFunction
    %IDQUADFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        weights
        angleConstraints
    end
    
    methods
        function id = IDMinQuadCableForcesConInteractionAngle(weights, angleConstraints)
            % 6p x 1 vector of weights (weight for each component of each
            % joint)
            id.weights = weights;
            id.angleConstraints = angleConstraints;
        end
        
        function [Qf, id_exit_type, id_info] = resolve(obj, dynamics)
            id_exit_type = IDExitType.NO_ERROR;
            % M\ddot{q} + C + G + F_{ext} = -J^T f (constraint)
            [A, b] = IDFunction.GetEoMConstraints(dynamics);     
            fmin = dynamics.cableDynamics.forcesMin;
            fmax = dynamics.cableDynamics.forcesMax;
            
            H = diag(obj.weights);
            f = zeros(length(obj.weights), 1);
            a = dynamics.P'*(dynamics.bodyDynamics.M_b*dynamics.q_ddot + dynamics.bodyDynamics.C_b - dynamics.bodyDynamics.G_b);
            F_T = dynamics.P'*dynamics.V';
            
            Q = cell(1,dynamics.numLinks);
            Q(:) = {zeros(dynamics.numCables, dynamics.numCables)};
            l = cell(1,dynamics.numLinks);
            l(:) = {zeros(dynamics.numLinks, 1)};
            r = cell(1,dynamics.numLinks);
            r(:) = {0};
            
            for k = 1:dynamics.numLinks
                mu_a = tan(obj.angleConstraints(k))^2;
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