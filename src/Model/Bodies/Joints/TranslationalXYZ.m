% 3D translation joint in the XYZ axes
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
classdef TranslationalXYZ < Joint
        
    properties (Constant = true)
        numDofs = 3;
        numVars = 3;
        q_default = [0; 0; 0];
        q_dot_default = [0; 0; 0];
        q_ddot_default = [0; 0; 0];
        q_lb = [-Inf; -Inf; -Inf];
        q_ub = [Inf; Inf; Inf];
    end
    
    properties (Dependent)
        % Translational DoFs
        x
        y
        z
        
        x_dot
        y_dot
        z_dot
    end
    
    methods
        % -------
        % Getters
        % -------
        function value = get.x(obj)
            value = obj.GetX(obj.q);
        end
        function value = get.x_dot(obj)
            value = obj.GetX(obj.q_dot);
        end
        function value = get.y(obj)
            value = obj.GetY(obj.q);
        end
        function value = get.y_dot(obj)
            value = obj.GetY(obj.q_dot);
        end
        function value = get.z(obj)
            value = obj.GetZ(obj.q);
        end
        function value = get.z_dot(obj)
            value = obj.GetZ(obj.q_dot);
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(~)
            R_pe = eye(3,3);
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(q)
            x = TranslationalXYZ.GetX(q);
            y = TranslationalXYZ.GetY(q);
            z = TranslationalXYZ.GetZ(q);
            r_rel = [x; y; z];
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(~)
            S = [eye(TranslationalXYZ.numDofs); zeros(3,3)];
        end
        
        % Generate the S gradient tensor
        function S_grad = RelVelocityMatrixGradient(~)
            S_grad = zeros(6,3,3);
        end
        
        % Generate the \dot{S} gradient tensor
        function S_dot_grad = RelVelocityMatrixDerivGradient(~,~)
            S_dot_grad = zeros(6,3,3);
        end
        
%         function S_dot = RelVelocityMatrixDeriv(~, ~)
%             S_dot = [zeros(6,3)];
%         end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(q)
            N_j =   zeros(TranslationalXYZ.numDofs,TranslationalXYZ.numDofs^2);
            A   =   zeros(6,TranslationalXYZ.numDofs);
        end
        
        % Generate the trajectories for the joint
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
            t = 0:time_step:total_time;
            [q(1,:), q_dot(1,:), q_ddot(1,:)] = Spline.QuinticInterpolation(q_s(1), q_s_d(1), q_s_dd(1), q_e(1), q_e_d(1), q_e_dd(1), t);
            [q(2,:), q_dot(2,:), q_ddot(2,:)] = Spline.QuinticInterpolation(q_s(2), q_s_d(2), q_s_dd(2), q_e(2), q_e_d(2), q_e_dd(2), t);
            [q(3,:), q_dot(3,:), q_ddot(3,:)] = Spline.QuinticInterpolation(q_s(3), q_s_d(3), q_s_dd(3), q_e(3), q_e_d(3), q_e_dd(3), t);
        end
        
        % Get variables from the gen coordinates
        function x = GetX(q)
            x = q(1);
        end
        function y = GetY(q)
            y = q(2);
        end
        function z = GetZ(q)
            z = q(3);
        end
    end
end

