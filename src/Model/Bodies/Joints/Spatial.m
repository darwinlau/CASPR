classdef Spatial < Joint
    %SphericalXYZ Joint definition for a spherical joint with Euler angles
    %xyz 
    properties
        translation
        orientation
    end
        
    properties (Constant = true)
        numDofs = TranslationalXYZ.numDofs + Spherical.numDofs;
        numVars = TranslationalXYZ.numVars + Spherical.numVars;
        q_default = [TranslationalXYZ.q_default; Spherical.q_default];
        q_dot_default = [TranslationalXYZ.q_dot_default; Spherical.q_dot_default];
        q_ddot_default = [TranslationalXYZ.q_ddot_default; Spherical.q_ddot_default];
        q_lb = [TranslationalXYZ.q_lb; Spherical.q_lb];
        q_ub = [TranslationalXYZ.q_ub; Spherical.q_ub];
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
        function j = Spatial()
            j.translation = TranslationalXYZ;
            j.orientation = Spherical;
        end
        
        function update(obj, q, q_dot, q_ddot)
            obj.translation.update(Spatial.GetTranslationQ(q), Spatial.GetTranslationQd(q_dot), Spatial.GetTranslationQd(q_ddot));
            obj.orientation.update(Spatial.GetOrientationQ(q), Spatial.GetOrientationQd(q_dot), Spatial.GetOrientationQd(q_ddot));
            update@Joint(obj, q, q_dot, q_ddot);
        end
        
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
        % The q vector for spatial is [x; y; z; e0; e1; e2; e3]
        % The q_d vector for spatial is [x_d; y_d; z_d; wx; wy; wz]
        function q_t = GetTranslationQ(q)
            q_t = q(1:TranslationalXYZ.numVars);
        end
        function q_t = GetTranslationQd(q_d)
            q_t = q_d(1:TranslationalXYZ.numVars);
        end
        function q_t = GetOrientationQ(q)
            q_t = q(TranslationalXYZ.numVars+1:Spatial.numVars);
        end
        function q_t_d = GetOrientationQd(q_d)
            q_t_d = q_d(TranslationalXYZ.numDofs+1:Spatial.numDofs);
        end
        
        function R_pe = RelRotationMatrix(q)
            R_pe = Spherical.RelRotationMatrix(Spatial.GetOrientationQ(q));
        end

        function r_rel = RelTranslationVector(q)
            r_rel = TranslationalXYZ.RelTranslationVector(Spatial.GetTranslationQ(q));
        end
        
        function S = RelVelocityMatrix(q)
            S = [TranslationalXYZ.RelVelocityMatrix(Spatial.GetTranslationQ(q)) Spherical.RelVelocityMatrix(Spatial.GetOrientationQ(q))];
        end
        
        function S_dot = RelVelocityMatrixDeriv(q, q_d)
            S_dot = [TranslationalXYZ.RelVelocityMatrixDeriv(Spatial.GetTranslationQ(q), Spatial.GetTranslationQd(q_d)) Spherical.RelVelocityMatrixDeriv(Spatial.GetOrientationQ(q), Spatial.GetOrientationQd(q_d))];
        end        
        
        % TO DO
        function [N_j,A] = QuadMatrix(~)
            N_j = zeros(Spatial.numDofs,Spatial.numDofs^2);
            A = zeros(6,Spatial.numDofs);
        end
        
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
            [q_trans, q_trans_dot, q_trans_ddot] = TranslationalXYZ.GenerateTrajectory( ...
                Spatial.GetTranslationQ(q_s), Spatial.GetTranslationQ(q_s_d), Spatial.GetTranslationQ(q_s_dd), ...
                Spatial.GetTranslationQ(q_e), Spatial.GetTranslationQ(q_e_d), Spatial.GetTranslationQ(q_e_dd), total_time, time_step);
            [q_orient, q_orient_dot, q_orient_ddot] = Spherical.GenerateTrajectory( ...
                Spatial.GetOrientationQ(q_s), Spatial.GetOrientationQd(q_s_d), Spatial.GetOrientationQd(q_s_dd), ...
                Spatial.GetOrientationQ(q_e), Spatial.GetOrientationQd(q_e_d), Spatial.GetOrientationQd(q_e_dd), total_time, time_step);
            q = [q_trans; q_orient];
            q_dot = [q_trans_dot; q_orient_dot];
            q_ddot = [q_trans_ddot; q_orient_ddot];
        end
    end
end