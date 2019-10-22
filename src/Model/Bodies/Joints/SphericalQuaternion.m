% Spherical joint (3-DOF) using the quaternion orientation representation
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
% THIS IS CURRENTLY NOT SUPPORTED IN CASPR
classdef SphericalQuaternion < JointBase
    
    properties (Constant = true)
        numDofs = 3;
        numVars = 4;
        q_default = [1; 0; 0; 0];   % 0 angle rotation about z axis
        q_dot_default = [0; 0; 0];
        q_ddot_default = [0; 0; 0];
        q_lb = [-1; -Inf; -Inf; -Inf];
        q_ub = [1; Inf; Inf; Inf];
    end
    
    properties (Dependent)
        % Quaternion of orientation
        e0      % real component of quaternion
        e1      % imaginary component of quaternion
        e2      % imaginary component of quaternion
        e3      % imaginary component of quaternion
        
        % Derivatives (use angular velocity components)
        wx
        wy
        wz
    end
    
    methods        
        % Update the joint information
        function update(obj, q, q_dot, q_ddot)
            if(isa(q, 'double') && norm(q) ~= 1)
                q_quat_norm = Quaternion(q(1), q(2), q(3), q(4)).normalise();
                CASPR_log.Assert(round(norm(q_quat_norm),5) == 1, 'Invalid q, norm of quaternion orientation must equal to one.');
            else
                q_quat_norm = Quaternion(q(1), q(2), q(3), q(4));
            end
            update@JointBase(obj, q_quat_norm.toVector(), q_dot, q_ddot);
        end
        
        
        function [q, q_dot, q_ddot] = generateTrajectoryLinearSpline(obj, q_s, q_e, time_vector)
            quat_0s = Quaternion(q_s(1), q_s(2), q_s(3), q_s(4));
            quat_0e = Quaternion(q_e(1), q_e(2), q_e(3), q_e(4));
                        
            [quat_sp, quat_sp_dot, quat_sp_ddot] = Quaternion.LinearInterpolation(quat_0s, quat_0e, time);      
            [q, q_dot, q_ddot] = Quaternion.quaternion_traj_to_q_traj(quat_sp, quat_sp_dot, quat_sp_ddot, quat_0s, time_vector);
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryCubicSpline(obj, q_s, q_s_d, q_e, q_e_d, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(isequal(q_s_d, zeros(n_dof, 1)) && isequal(q_e_d, zeros(n_dof, 1)), ...
                'Non-zero joint velocity are currently not supported for quaternion splines');                 
            quat_0s = Quaternion(q_s(1), q_s(2), q_s(3), q_s(4));
            quat_0e = Quaternion(q_e(1), q_e(2), q_e(3), q_e(4));
                        
            [quat_sp, quat_sp_dot, quat_sp_ddot] = Quaternion.CubicInterpolation(quat_0s, quat_0e, time);      
            [q, q_dot, q_ddot] = Quaternion.quaternion_traj_to_q_traj(quat_sp, quat_sp_dot, quat_sp_ddot, quat_0s, time_vector);
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryQuinticSpline(obj, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(isequal(q_s_d, zeros(n_dof, 1)) && isequal(q_e_d, zeros(n_dof, 1)) && isequal(q_s_dd, zeros(n_dof, 1)) && isequal(q_e_dd, zeros(n_dof, 1)), ...
                'Non-zero joint velocity and accelerations are currently not supported for quaternion splines');     
            quat_0s = Quaternion(q_s(1), q_s(2), q_s(3), q_s(4));
            quat_0e = Quaternion(q_e(1), q_e(2), q_e(3), q_e(4));
                        
            [quat_sp, quat_sp_dot, quat_sp_ddot] = Quaternion.QuinticInterpolation(quat_0s, quat_0e, time);      
            [q, q_dot, q_ddot] = Quaternion.quaternion_traj_to_q_traj(quat_sp, quat_sp_dot, quat_sp_ddot, quat_0s, time_vector);
        end
        
        % -------
        % Getters
        % -------
        function value = get.e0(obj)
            value = obj.GetE0(obj.q);
        end
        function value = get.e1(obj)
            value = obj.GetE1(obj.q);
        end
        function value = get.e2(obj)
            value = obj.GetE2(obj.q);
        end
        function value = get.e3(obj)
            value = obj.GetE3(obj.q);
        end
        
        function value = get.wx(obj)
            value = obj.GetWx(obj.q_dot);
        end
        function value = get.wy(obj)
            value = obj.GetWy(obj.q_dot);
        end
        function value = get.wz(obj)
            value = obj.GetWz(obj.q_dot);
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            e0 = SphericalQuaternion.GetE0(q);
            e1 = SphericalQuaternion.GetE1(q);
            e2 = SphericalQuaternion.GetE2(q);
            e3 = SphericalQuaternion.GetE3(q);
            
            R_pe = Quaternion.ToRotationMatrix(Quaternion(e0, e1, e2, e3));
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(~)
            S = [zeros(3,3); eye(3, 3)];
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(~)
            S_grad = zeros(6,3,3);
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(~,~)
            S_dot_grad = zeros(6,3,3);
        end
        
%         function S_dot = RelVelocityMatrixDeriv(~, ~)
%             S_dot = zeros(6, 3);
%         end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            N_j = zeros(SphericalQuaternion.numDofs,SphericalQuaternion.numDofs^2);
            A = zeros(6,SphericalQuaternion.numDofs);
        end
        
        % Perform a simple first order integral
        function q = QIntegrate(q0, q_dot, dt)
            q0_quat = Quaternion(q0(1), q0(2), q0(3), q0(4));
            w_quat = Quaternion(0, q_dot(1), q_dot(2), q_dot(3));
            q_quat = q0_quat * exp(0.5*w_quat*dt);
            q = q_quat.toVector();
        end
        
        % Obtain the derivative
        function q_deriv = QDeriv(q, q_dot)
            w_quat = Quaternion(0, q_dot(1), q_dot(2), q_dot(3));
            q_quat = Quaternion(q(1), q(2), q(3), q(4));
            q_deriv_quat = 1/2 * w_quat * q_quat;
            q_deriv = [q_deriv_quat.q0; q_deriv_quat.q1; q_deriv_quat.q2; q_deriv_quat.q3];
        end
        
        % Generate the trajectory for the joint
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, ~, ~, q_e, ~, ~, total_time, time_step)
            time = 0:time_step:total_time;
                        
            quat_0s = Quaternion(q_s(1), q_s(2), q_s(3), q_s(4));
            quat_0e = Quaternion(q_e(1), q_e(2), q_e(3), q_e(4));
                        
            [quat_sp, quat_sp_dot, quat_sp_ddot] = Quaternion.GenerateInterpolation(quat_0s, quat_0e, time);
            
            q = zeros(4, length(time));
            q_dot = zeros(3, length(time));
            q_ddot = zeros(3, length(time));
            
            for t = 1:length(time)
                q_0p = quat_sp(t)*quat_0s;
                q_0p_dot = quat_sp_dot(t)*quat_0s;
                                
                w_quat = 2*q_0p_dot*inv(q_0p);
                w_quat_d = 2*(quat_sp_ddot(t)*inv(q_0p) + q_0p_dot*inv(q_0p_dot));
                
                q(1,t) = q_0p.q0;
                q(2,t) = q_0p.q1;
                q(3,t) = q_0p.q2;
                q(4,t) = q_0p.q3;
                q_dot(1,t) = w_quat.q1;
                q_dot(2,t) = w_quat.q2;
                q_dot(3,t) = w_quat.q3;
                q_ddot(1,t) = w_quat_d.q1;
                q_ddot(2,t) = w_quat_d.q2;
                q_ddot(3,t) = w_quat_d.q3;
            end
        end
        
        % Get variables from the gen coordinates
        function e0 = GetE0(q)
            e0 = q(1);
        end
        function e1 = GetE1(q)
            e1 = q(2);
        end
        function e2 = GetE2(q)
            e2 = q(3);
        end
        function e3 = GetE3(q)
            e3 = q(4);
        end
        
        function wx = GetWx(q_dot)
            wx = q_dot(1);
        end
        function wy = GetWy(q_dot)
            wy = q_dot(2);
        end
        function wz = GetWz(q_dot)
            wz = q_dot(3);
        end
    end
    
    methods (Static, Access = private)
        function [q, q_dot, q_ddot] = quaternion_traj_to_q_traj(quat_sp, quat_sp_dot, quat_sp_ddot, quat_0s, time_vector)            
            q = zeros(4, length(time_vector));
            q_dot = zeros(3, length(time_vector));
            q_ddot = zeros(3, length(time_vector));
            
            for t = 1:length(time_vector)
                q_0p = quat_sp(t)*quat_0s;
                q_0p_dot = quat_sp_dot(t)*quat_0s;
                                
                w_quat = 2*q_0p_dot*inv(q_0p);
                w_quat_d = 2*(quat_sp_ddot(t)*inv(q_0p) + q_0p_dot*inv(q_0p_dot));
                
                q(1,t) = q_0p.q0;
                q(2,t) = q_0p.q1;
                q(3,t) = q_0p.q2;
                q(4,t) = q_0p.q3;
                q_dot(1,t) = w_quat.q1;
                q_dot(2,t) = w_quat.q2;
                q_dot(3,t) = w_quat.q3;
                q_ddot(1,t) = w_quat_d.q1;
                q_ddot(2,t) = w_quat_d.q2;
                q_ddot(3,t) = w_quat_d.q3;
            end
        end
    end
end

