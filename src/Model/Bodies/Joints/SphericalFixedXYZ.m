% Spherical joint (3-DOF) using the XYZ-Fixed representation
%
% Author        : Darwin LAU
% Created       : 2012
% Description   :
classdef SphericalFixedXYZ < JointBase
    properties (Constant = true)
        numDofs = 3;
        numVars = 3;
        q_default = [0; 0; 0];
        q_dot_default = [0; 0; 0];
        q_ddot_default = [0; 0; 0];
        q_lb = [-pi; -pi; -pi];
        q_ub = [pi; pi; pi];
        q_dofType = [DoFType.ROTATION; DoFType.ROTATION; DoFType.ROTATION];
    end
    
    properties (Dependent)
        alpha
        beta
        gamma
        alpha_dot
        beta_dot
        gamma_dot
    end
    
    methods
        function [q, q_dot, q_ddot] = generateTrajectoryLinearSpline(obj, q_s, q_e, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e), ...
                'Length of input states are different to the number of DoFs');            
            R_0s = SphericalFixedXYZ.RelRotationMatrix(q_s);
            quat_s = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_s));
            quat_e = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_e));            
            [quat, quat_dot, quat_ddot] = Quaternion.LinearInterpolation(quat_s, quat_e, time_vector);            
            [q, q_dot, q_ddot] = SphericalFixedXYZ.quaternion_traj_to_q_traj(quat, quat_dot, quat_ddot, R_0s, time_vector);
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryCubicSpline(obj, q_s, q_s_d, q_e, q_e_d, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e) && n_dof == length(q_s_d) && n_dof == length(q_e_d), ...
                'Length of input states are different to the number of DoFs');
            CASPR_log.Assert(isequal(q_s_d, zeros(n_dof, 1)) && isequal(q_e_d, zeros(n_dof, 1)), ...
                'Non-zero joint velocity are currently not supported for spherical joint splines');                 
            R_0s = SphericalFixedXYZ.RelRotationMatrix(q_s);
            quat_s = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_s));
            quat_e = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_e));            
            [quat, quat_dot, quat_ddot] = Quaternion.CubicInterpolation(quat_s, quat_e, time_vector);            
            [q, q_dot, q_ddot] = SphericalFixedXYZ.quaternion_traj_to_q_traj(quat, quat_dot, quat_ddot, R_0s, time_vector);
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryQuinticSpline(obj, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, time_vector)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e) && n_dof == length(q_s_d) && n_dof == length(q_e_d) && n_dof == length(q_s_dd) && n_dof == length(q_e_dd), ...
                'Length of input states are different to the number of DoFs');
            CASPR_log.Assert(isequal(q_s_d, zeros(n_dof, 1)) && isequal(q_e_d, zeros(n_dof, 1)) && isequal(q_s_dd, zeros(n_dof, 1)) && isequal(q_e_dd, zeros(n_dof, 1)), ...
                'Non-zero joint velocity and accelerations are currently not supported for spherical joint splines');     
            
            R_0s = SphericalFixedXYZ.RelRotationMatrix(q_s);
            quat_s = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_s));
            quat_e = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_e));
            
            [quat, quat_dot, quat_ddot] = Quaternion.QuinticInterpolation(quat_s, quat_e, time_vector);
            
            [q, q_dot, q_ddot] = SphericalFixedXYZ.quaternion_traj_to_q_traj(quat, quat_dot, quat_ddot, R_0s, time_vector);
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryParabolicBlend(obj, q_s, q_e, time_vector, blend_time)
            n_dof = obj.numDofs;
            CASPR_log.Assert(n_dof == length(q_s) && n_dof == length(q_e), ...
                'Length of input states are different to the number of DoFs');            
            R_0s = SphericalFixedXYZ.RelRotationMatrix(q_s);
            quat_s = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_s));
            quat_e = Quaternion.FromRotationMatrix(SphericalFixedXYZ.RelRotationMatrix(q_e));            
            [quat, quat_dot, quat_ddot] = Quaternion.ParabolicBlend(quat_s, quat_e, time_vector, blend_time);            
            [q, q_dot, q_ddot] = SphericalFixedXYZ.quaternion_traj_to_q_traj(quat, quat_dot, quat_ddot, R_0s, time_vector);
        end
        
        % -------
        % Getters
        % -------
        function value = get.alpha(obj)
            value = obj.GetAlpha(obj.q);
        end
        function value = get.alpha_dot(obj)
            value = obj.GetAlpha(obj.q_dot);
        end
        function value = get.beta(obj)
            value = obj.GetBeta(obj.q);
        end
        function value = get.beta_dot(obj)
            value = obj.GetBeta(obj.q_dot);
        end
        function value = get.gamma(obj)
            value = obj.GetGamma(obj.q);
        end
        function value = get.gamma_dot(obj)
            value = obj.GetGamma(obj.q_dot);
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            alpha = SphericalFixedXYZ.GetAlpha(q);
            beta = SphericalFixedXYZ.GetBeta(q);
            gamma = SphericalFixedXYZ.GetGamma(q);
            R_01 = RevoluteZ.RelRotationMatrix(gamma);
            R_12 = RevoluteY.RelRotationMatrix(beta);
            R_23 = RevoluteX.RelRotationMatrix(alpha);
            R_pe = R_01*R_12*R_23;
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(q)
            a = SphericalFixedXYZ.GetAlpha(q);
            b = SphericalFixedXYZ.GetBeta(q);
            S = [zeros(3,3); 1 0 -sin(b); 0 cos(a) sin(a)*cos(b); 0 -sin(a) cos(a)*cos(b)];
        end
        
        % Generate the S gradient tensor
        function S_grad = RelVelocityMatrixGradient(q)
            a               =   SphericalFixedXYZ.GetAlpha(q);
            b               =   SphericalFixedXYZ.GetBeta(q);
            S_grad          =   MatrixOperations.Initialise([6,3,3],isa(q, 'sym'));
            S_grad(:,:,1)   =   [zeros(4,3);0,-sin(a),cos(a)*cos(b);0,-cos(a),-sin(a)*cos(b)];
            S_grad(:,:,2)   =   [zeros(3,3);0,0,-cos(b);0,0,-sin(a)*sin(b);0,0,-cos(a)*sin(b)];
        end
        
        % Generate the \dot{S} gradient tensor
        function S_dot_grad = RelVelocityMatrixDerivGradient(q,q_dot)
            a               =   SphericalFixedXYZ.GetAlpha(q);
            b               =   SphericalFixedXYZ.GetBeta(q);
            a_d = SphericalFixedXYZ.GetAlpha(q_dot);
            b_d = SphericalFixedXYZ.GetBeta(q_dot);
            S_dot_grad          =   MatrixOperations.Initialise([6,3,3],isa(q, 'sym'));
            S_dot_grad(:,:,1)   =   [zeros(4,3);0,-a_d*cos(a),-a_d*sin(a)*cos(b) - b_d*cos(a)*sin(b);0,a_d*sin(a),-a_d*cos(a)*cos(b) + b_d*sin(a)*sin(b)];
            S_dot_grad(:,:,2)   =   [zeros(3,3);0,0,b_d*sin(b);0,0,-a_d*sin(a)*cos(b) - b_d*sin(a)*cos(b);0,0,a_d*sin(a)*sin(b) - b_d*cos(a)*cos(b)];
        end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(q)
            a = SphericalFixedXYZ.GetAlpha(q);
            b = SphericalFixedXYZ.GetBeta(q);
            N_j = [[0,0,0;0,0,-cos(b)/2;0,-cos(b)/2,0],...
                [0,-sin(a)/2,cos(a)*cos(b)/2;-sin(a)/2,0,-sin(a)*sin(b)/2;cos(a)*cos(b)/2,-sin(a)*sin(b)/2,0],...
                [0,-cos(a)/2,-sin(a)*cos(b)/2;-cos(a)/2,0,-cos(a)*sin(b)/2;-sin(a)*cos(b)/2,-cos(a)*sin(b)/2,0]];
            A = [zeros(3);eye(3)];
        end
        
        % Get variables from the gen coordinates
        function alpha = GetAlpha(q)
            alpha = q(1);
        end
        function beta = GetBeta(q)
            beta = q(2);
        end
        function gamma = GetGamma(q)
            gamma = q(3);
        end
        
    end
    
    methods (Static, Access = private)
        % Conversions between representations
        function [a, b, g] = rotation_matrix_to_angles(R_0p)
            % Extract the angles from the rotation matrix
            b = -asin(R_0p(3,1));
            g = atan2(R_0p(2,1), R_0p(1,1));
            a = atan2(R_0p(3,2), R_0p(3,3));
            a = round(a, 10);
            b = round(b, 10);
            g = round(g, 10);
        end
        
        function [a_d, b_d, g_d] = rotation_matrix_to_angles_deriv(a, b, g, R_0p_dot)
            b_d = -R_0p_dot(3,1)/cos(b);
            g_d = (-b_d*sin(b)*cos(g)-R_0p_dot(1,1))/(cos(b)*sin(g));
            a_d = (-b_d*cos(a)*sin(b)-R_0p_dot(3,3))/(sin(a)*cos(b));
            a_d = round(a_d, 10);
            b_d = round(b_d, 10);
            g_d = round(g_d, 10);
        end
        
        function [a_dd, b_dd, g_dd] = rotation_matrix_to_angles_deriv_d(a, b, g, a_d, b_d, g_d, R_0p_ddot)
            b_dd = (-R_0p_ddot(3,1)+b_d^2*sin(b))/cos(b);
            g_dd = (-b_dd*sin(b)*cos(g)-b_d*(b_d*cos(b)*cos(g)-g_d*sin(b)*sin(g))-g_d*(-b_d*sin(b)*sin(g)+g_d*cos(b)*cos(g))-R_0p_ddot(1,1))/(cos(b)*sin(g));
            a_dd = (-a_d*(a_d*cos(a)*cos(b)-b_d*sin(a)*sin(b))-b_dd*cos(a)*sin(b)-b_d*(-a_d*sin(a)*sin(b)+b_d*cos(a)*cos(b))-R_0p_ddot(3,3))/(sin(a)*cos(b));
            a_dd = round(a_dd, 10);
            b_dd = round(b_dd, 10);
            g_dd = round(g_dd, 10);
        end
        
        function [q, q_dot, q_ddot] = quaternion_traj_to_q_traj(quat, quat_dot, quat_ddot, R_0s, time_vector)
            q = zeros(3, length(time_vector));
            q_dot = zeros(3, length(time_vector));
            q_ddot = zeros(3, length(time_vector));
            
            for t = 1:length(time_vector)
                R_sp = Quaternion.ToRotationMatrix(quat(t));
                % Rotation matrix and its derivatives for q at this instance in time
                R_0p = R_0s*R_sp;
                R_sp_dot = Quaternion.ToRotationMatrixDeriv(quat(t), quat_dot(t));
                R_0p_dot = R_0s*R_sp_dot;
                R_sp_ddot = Quaternion.ToRotationMatrixDoubleDeriv(quat(t), quat_dot(t), quat_ddot(t));
                R_0p_ddot = R_0s*R_sp_ddot;
                
                [a, b, g] = SphericalFixedXYZ.rotation_matrix_to_angles(R_0p);                
                [a_d, b_d, g_d] = SphericalFixedXYZ.rotation_matrix_to_angles_deriv(a, b, g, R_0p_dot);                
                if (~isfinite(a_d))
                    if (t>1)
                        a_d = (a-q(1,t-1))/(time_vector(t)-time_vector(t-1));
                    else
                        a_d = 0;
                    end
                end
                if (~isfinite(b_d))
                    if (t>1)
                        b_d = (b-q(2,t-1))/(time_vector(t)-time_vector(t-1));
                    else
                        b_d = 0;
                    end
                end
                if (~isfinite(g_d))
                    if (t>1)
                        g_d = (g-q(3,t-1))/(time_vector(t)-time_vector(t-1));
                    else
                        g_d = 0;
                    end
                end
                
                [a_dd, b_dd, g_dd] = SphericalFixedXYZ.rotation_matrix_to_angles_deriv_d(a, b, g, a_d, b_d, g_d, R_0p_ddot);
                if (~isfinite(a_dd))
                    if (t>1)
                        a_dd = (a_d-q_dot(1,t-1))/(time_vector(t)-time_vector(t-1));
                    else
                        a_dd = 0;
                    end
                end
                if (~isfinite(b_dd))
                    if (t>1)
                        b_dd = (b_d-q_dot(2,t-1))/(time_vector(t)-time_vector(t-1));
                    else
                        b_dd = 0;
                    end
                end
                if (~isfinite(g_dd))
                    if (t>1)
                        g_dd = (g_d-q_dot(3,t-1))/(time_vector(t)-time_vector(t-1));
                    else
                        g_dd = 0;
                    end
                end
                
                q(1,t) = a;
                q(2,t) = b;
                q(3,t) = g;
                q_dot(1,t) = a_d;
                q_dot(2,t) = b_d;
                q_dot(3,t) = g_d;
                q_ddot(1,t) = a_dd;
                q_ddot(2,t) = b_dd;
                q_ddot(3,t) = g_dd;
            end
        end
    end
end

