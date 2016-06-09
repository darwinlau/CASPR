% Spatial (6-DOF) joint using the XYZ-Euler orientation representation
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
classdef SpatialEulerXYZ < Joint
    properties
        translation
        orientation
    end
        
    properties (Constant = true)
        numDofs = TranslationalXYZ.numDofs + SphericalEulerXYZ.numDofs;
        numVars = TranslationalXYZ.numVars + SphericalEulerXYZ.numVars;
        q_default = [TranslationalXYZ.q_default; SphericalEulerXYZ.q_default];
        q_dot_default = [TranslationalXYZ.q_dot_default; SphericalEulerXYZ.q_dot_default];
        q_ddot_default = [TranslationalXYZ.q_ddot_default; SphericalEulerXYZ.q_ddot_default];
        q_lb = [TranslationalXYZ.q_lb; SphericalEulerXYZ.q_lb];
        q_ub = [TranslationalXYZ.q_ub; SphericalEulerXYZ.q_ub];
    end
    
    properties (Dependent)
        q_translation
        q_orientation
        
        x
        y
        z
        x_dot
        y_dot
        z_dot
        
        alpha
        beta
        gamma
        alpha_dot
        beta_dot
        gamma_dot
    end
    
    methods
        % Constructors
        function j = SpatialEulerXYZ()
            j.translation = TranslationalXYZ;
            j.orientation = SphericalEulerXYZ;
        end
        
        % Update the joint space
        function update(obj, q, q_dot, q_ddot)
            obj.translation.update(SpatialEulerXYZ.GetTranslationQ(q), SpatialEulerXYZ.GetTranslationQd(q_dot), SpatialEulerXYZ.GetTranslationQd(q_ddot));
            obj.orientation.update(SpatialEulerXYZ.GetOrientationQ(q), SpatialEulerXYZ.GetOrientationQd(q_dot), SpatialEulerXYZ.GetOrientationQd(q_ddot));
            update@Joint(obj, q, q_dot, q_ddot);
        end
        
        % -------
        % Getters
        % -------
        function value = get.x(obj)
            value = obj.translation.x;
        end
        function value = get.y(obj)
            value = obj.translation.y;
        end
        function value = get.z(obj)
            value = obj.translation.z;
        end
        function value = get.x_dot(obj)
            value = obj.translation.x_dot;
        end
        function value = get.y_dot(obj)
            value = obj.translation.y_dot;
        end
        function value = get.z_dot(obj)
            value = obj.translation.z_dot;
        end
        function value = get.alpha(obj)
            value = obj.orientation.alpha;
        end
        function value = get.beta(obj)
            value = obj.orientation.beta;
        end
        function value = get.gamma(obj)
            value = obj.orientation.gamma;
        end
        function value = get.alpha_dot(obj)
            value = obj.orientation.alpha_dot;
        end
        function value = get.beta_dot(obj)
            value = obj.orientation.beta_dot;
        end
        function value = get.gamma_dot(obj)
            value = obj.orientation.gamma_dot;
        end
        
    end
    
    methods (Static)
        % The q vector for SpatialEulerXYZ is [x; y; z; e0; e1; e2; e3]
        % The q_d vector for SpatialEulerXYZ is [x_d; y_d; z_d; wx; wy; wz]
        function q_t = GetTranslationQ(q)
            q_t = q(1:TranslationalXYZ.numVars);
        end
        function q_t = GetTranslationQd(q_d)
            q_t = q_d(1:TranslationalXYZ.numVars);
        end
        function q_t = GetOrientationQ(q)
            q_t = q(TranslationalXYZ.numVars+1:SpatialEulerXYZ.numVars);
        end
        function q_t_d = GetOrientationQd(q_d)
            q_t_d = q_d(TranslationalXYZ.numDofs+1:SpatialEulerXYZ.numDofs);
        end
        
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            R_pe = SphericalEulerXYZ.RelRotationMatrix(SpatialEulerXYZ.GetOrientationQ(q));
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(q)
            r_rel = TranslationalXYZ.RelTranslationVector(SpatialEulerXYZ.GetTranslationQ(q));
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(q)
            S = [TranslationalXYZ.RelVelocityMatrix(SpatialEulerXYZ.GetTranslationQ(q)) ...
                SphericalEulerXYZ.RelVelocityMatrix(SpatialEulerXYZ.GetOrientationQ(q))];
        end
        
        % Generate the S gradient tensor
        function S_grad = RelVelocityMatrixGradient(q)
            S_grad              =   MatrixOperations.Initialise([6,6,6],isa(q, 'sym'));
            S_grad(:,4:6,4:6)   =  SphericalEulerXYZ.RelVelocityMatrixGradient(SpatialEulerXYZ.GetOrientationQ(q));
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(q,q_dot)
            S_dot_grad              =   MatrixOperations.Initialise([6,6,6],isa(q, 'sym'));
            S_dot_grad(:,4:6,4:6)   =  SphericalEulerXYZ.RelVelocityMatrixDerivGradient(SpatialEulerXYZ.GetOrientationQ(q),SpatialEulerXYZ.GetOrientationQd(q_dot));
        end
        
%         function S_dot = RelVelocityMatrixDeriv(q, q_d)
%             S_dot = [TranslationalXYZ.RelVelocityMatrixDeriv(SpatialEulerXYZ.GetTranslationQ(q), SpatialEulerXYZ.GetTranslationQd(q_d)) ...
%                 SphericalEulerXYZ.RelVelocityMatrixDeriv(SpatialEulerXYZ.GetOrientationQ(q), SpatialEulerXYZ.GetOrientationQd(q_d))];
%         end
        
        % The derivative matrix
        function q_deriv = QDeriv(q, q_d)
            q_deriv = [TranslationalXYZ.QDeriv(SpatialEulerXYZ.GetTranslationQ(q), SpatialEulerXYZ.GetTranslationQd(q_d)); ...
                SphericalEulerXYZ.QDeriv(SpatialEulerXYZ.GetOrientationQ(q), SpatialEulerXYZ.GetOrientationQd(q_d))];
        end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            N_j = zeros(SpatialEulerXYZ.numDofs,SpatialEulerXYZ.numDofs^2);
            A = zeros(6,SpatialEulerXYZ.numDofs);
        end
        
        % Generate the trajectory for this joint
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
            [q_trans, q_trans_dot, q_trans_ddot] = TranslationalXYZ.GenerateTrajectory( ...
                SpatialEulerXYZ.GetTranslationQ(q_s), SpatialEulerXYZ.GetTranslationQ(q_s_d), SpatialEulerXYZ.GetTranslationQ(q_s_dd), ...
                SpatialEulerXYZ.GetTranslationQ(q_e), SpatialEulerXYZ.GetTranslationQ(q_e_d), SpatialEulerXYZ.GetTranslationQ(q_e_dd), total_time, time_step);
            [q_orient, q_orient_dot, q_orient_ddot] = SphericalEulerXYZ.GenerateTrajectory( ...
                SpatialEulerXYZ.GetOrientationQ(q_s), SpatialEulerXYZ.GetOrientationQd(q_s_d), SpatialEulerXYZ.GetOrientationQd(q_s_dd), ...
                SpatialEulerXYZ.GetOrientationQ(q_e), SpatialEulerXYZ.GetOrientationQd(q_e_d), SpatialEulerXYZ.GetOrientationQd(q_e_dd), total_time, time_step);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
    end
end