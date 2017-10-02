% SpatialQuaternion (6-DOF) joint using the quaternion orientation representation
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
% THIS IS CURRENTLY NOT SUPPORTED IN CASPR
classdef SpatialQuaternion < JointBase
    properties
        translation
        orientation
    end
        
    properties (Constant = true)
        numDofs = TranslationalXYZ.numDofs + SphericalQuaternion.numDofs;
        numVars = TranslationalXYZ.numVars + SphericalQuaternion.numVars;
        q_default = [TranslationalXYZ.q_default; SphericalQuaternion.q_default];
        q_dot_default = [TranslationalXYZ.q_dot_default; SphericalQuaternion.q_dot_default];
        q_ddot_default = [TranslationalXYZ.q_ddot_default; SphericalQuaternion.q_ddot_default];
        q_lb = [TranslationalXYZ.q_lb; SphericalQuaternion.q_lb];
        q_ub = [TranslationalXYZ.q_ub; SphericalQuaternion.q_ub];
    end
    
    properties (Dependent)
        x
        y
        z
        x_dot
        y_dot
        z_dot
        
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
        % Constructor for the class
        function j = SpatialQuaternion()
            j.translation = TranslationalXYZ;
            j.orientation = SphericalQuaternion;
        end
        
        % Update the joint space information
        function update(obj, q, q_dot, q_ddot)
            obj.translation.update(SpatialQuaternion.GetTranslationQ(q), SpatialQuaternion.GetTranslationQd(q_dot), SpatialQuaternion.GetTranslationQd(q_ddot));
            obj.orientation.update(SpatialQuaternion.GetOrientationQ(q), SpatialQuaternion.GetOrientationQd(q_dot), SpatialQuaternion.GetOrientationQd(q_ddot));
            update@JointBase(obj, q, q_dot, q_ddot);
        end
        
        % Generate the trajectory functions
        function [q, q_dot, q_ddot] = generateTrajectoryLinearSpline(obj, q_s, q_e, time_vector)
            [q_trans, q_trans_dot, q_trans_ddot] = obj.translation.generateTrajectoryLinearSpline( ...
                obj.GetTranslationQ(q_s), ...
                obj.GetTranslationQ(q_e), time_vector);
            [q_orient, q_orient_dot, q_orient_ddot] = obj.orientation.generateTrajectoryLinearSpline( ...
                obj.GetOrientationQ(q_s), ...
                obj.GetOrientationQ(q_e), time_vector);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryCubicSpline(obj, q_s, q_s_d, q_e, q_e_d, time_vector)
            [q_trans, q_trans_dot, q_trans_ddot] = obj.translation.generateTrajectoryCubicSpline( ...
                obj.GetTranslationQ(q_s), obj.GetTranslationQd(q_s_d), ...
                obj.GetTranslationQ(q_e), obj.GetTranslationQd(q_e_d), time_vector);
            [q_orient, q_orient_dot, q_orient_ddot] = obj.orientation.generateTrajectoryCubicSpline( ...
                obj.GetOrientationQ(q_s), obj.GetOrientationQd(q_s_d), ...
                obj.GetOrientationQ(q_e), obj.GetOrientationQd(q_s_e), time_vector);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
        
        function [q, q_dot, q_ddot] = generateTrajectoryQuinticSpline(obj, q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, time_vector)
            [q_trans, q_trans_dot, q_trans_ddot] = obj.translation.generateTrajectoryQuinticSpline( ...
                obj.GetTranslationQ(q_s), obj.GetTranslationQd(q_s_d), obj.GetTranslationQd(q_s_dd), ...
                obj.GetTranslationQ(q_e), obj.GetTranslationQd(q_e_d), obj.GetTranslationQd(q_e_dd), time_vector);
            [q_orient, q_orient_dot, q_orient_ddot] = obj.orientation.generateTrajectoryQuinticSpline( ...
                obj.GetOrientationQ(q_s), obj.GetOrientationQd(q_s_d), obj.GetOrientationQd(q_s_dd), ...
                obj.GetOrientationQ(q_e), obj.GetOrientationQd(q_e_d), obj.GetOrientationQd(q_e_dd), time_vector);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
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
        
        function value = get.e0(obj)
            value = obj.orientation.e0;
        end
        function value = get.e1(obj)
            value = obj.orientation.e1;
        end
        function value = get.e2(obj)
            value = obj.orientation.e2;
        end
        function value = get.e3(obj)
            value = obj.orientation.e3;
        end
        function value = get.wx(obj)
            value = obj.orientation.wx;
        end
        function value = get.wy(obj)
            value = obj.orientation.wy;
        end
        function value = get.wz(obj)
            value = obj.orientation.wz;
        end
    end
    
    methods (Static)
        % The q vector for SpatialQuaternion is [x; y; z; e0; e1; e2; e3]
        % The q_d vector for SpatialQuaternion is [x_d; y_d; z_d; wx; wy; wz]
        function q_t = GetTranslationQ(q)
            q_t = q(1:TranslationalXYZ.numVars);
        end
        function q_t = GetTranslationQd(q_d)
            q_t = q_d(1:TranslationalXYZ.numVars);
        end
        function q_t = GetOrientationQ(q)
            q_t = q(TranslationalXYZ.numVars+1:SpatialQuaternion.numVars);
        end
        function q_t_d = GetOrientationQd(q_d)
            q_t_d = q_d(TranslationalXYZ.numDofs+1:SpatialQuaternion.numDofs);
        end
        
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            R_pe = SphericalQuaternion.RelRotationMatrix(SpatialQuaternion.GetOrientationQ(q));
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(q)
            r_rel = TranslationalXYZ.RelTranslationVector(SpatialQuaternion.GetTranslationQ(q));
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(q)
            S = [TranslationalXYZ.RelVelocityMatrix(SpatialQuaternion.GetTranslationQ(q)) SphericalQuaternion.RelVelocityMatrix(SpatialQuaternion.GetOrientationQ(q))];
        end
        
        % Generate the S gradient tensor
        function S_grad = RelVelocityMatrixGradient(~)
            S_grad = zeros(6,6,6);
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(~,~)
            S_dot_grad = zeros(6,6,6);
        end
        
%         function S_dot = RelVelocityMatrixDeriv(q, q_d)
%             S_dot = [TranslationalXYZ.RelVelocityMatrixDeriv(SpatialQuaternion.GetTranslationQ(q), SpatialQuaternion.GetTranslationQd(q_d)) SphericalQuaternion.RelVelocityMatrixDeriv(SpatialQuaternion.GetOrientationQ(q), SpatialQuaternion.GetOrientationQd(q_d))];
%         end        
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            N_j = zeros(SpatialQuaternion.numDofs,SpatialQuaternion.numDofs^2);
            A = zeros(6,SpatialQuaternion.numDofs);
        end
        
        % Generate the derivative
        function q_deriv = QDeriv(q, q_d)
            q_deriv = [TranslationalXYZ.QDeriv(SpatialQuaternion.GetTranslationQ(q), SpatialQuaternion.GetTranslationQd(q_d)); SphericalQuaternion.QDeriv(SpatialQuaternion.GetOrientationQ(q), SpatialQuaternion.GetOrientationQd(q_d))];
        end
    end
end