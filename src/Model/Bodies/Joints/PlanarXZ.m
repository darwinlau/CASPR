% Planar joint in the XZ-plane
%
% Author        : Benji ZHANG
% Created       : 2017
% Description   : 
classdef PlanarXZ < JointBase        
    properties (Constant = true)
        numDofs = 3;
        numVars = 3;
        q_default = [0; 0; 0];
        q_dot_default = [0; 0; 0];
        q_ddot_default = [0; 0; 0];
        q_lb = [-Inf; -Inf; -pi];
        q_ub = [Inf; Inf; pi];
    end    
    
    properties (Dependent)
        x
        z
        theta
        x_dot
        z_dot
        theta_dot
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
        function value = get.z(obj)
            value = obj.GetZ(obj.q);
        end
        function value = get.z_dot(obj)
            value = obj.GetZ(obj.q_dot);
        end
        function value = get.theta(obj)
            value = obj.GetTheta(obj.q);
        end
        function value = get.theta_dot(obj)
            value = obj.GetTheta(obj.q_dot);
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix %%%%%%%%%
        function R_pe = RelRotationMatrix(q)
            theta = PlanarXZ.GetTheta(q);
            R_pe = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
        end

        % Get the relative translation vector %%%%%%%%%%%
        function r_rel = RelTranslationVector(q)
            x = PlanarXZ.GetX(q);
            z = PlanarXZ.GetZ(q);
            r_rel = [x; 0; z];
        end

        % Generate the S matrix %%%%%%%%%
        function S = RelVelocityMatrix(~)
            S = [1 0 0; 0 0 0; 0 1 0; 0 0 0; 0 0 1; 0 0 0];
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(~)
            S_grad = zeros(6,3,3);
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(~,~)
            S_dot_grad = zeros(6,3,3);
        end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            N_j = zeros(PlanarXZ.numDofs,PlanarXZ.numDofs^2);
            A = zeros(6,PlanarXZ.numDofs);
        end
        
        % Get variables from the gen coordinates
        function x = GetX(q)
            x = q(1);
        end
        function z = GetZ(q)
            z = q(2);
        end
        function theta = GetTheta(q)
            theta = q(3);
        end
        
        % Should this be a separate function
        function theta_d = GetThetaDot(q_dot)
            theta_d = q_dot(3);
        end
        
        % Generate trajectories
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
            t = 0:time_step:total_time;
            [q(1,:), q_dot(1,:), q_ddot(1,:)] = Spline.QuinticInterpolation(q_s(1), q_s_d(1), q_s_dd(1), q_e(1), q_e_d(1), q_e_dd(1), t);
            [q(2,:), q_dot(2,:), q_ddot(2,:)] = Spline.QuinticInterpolation(q_s(2), q_s_d(2), q_s_dd(2), q_e(2), q_e_d(2), q_e_dd(2), t);
            [q(3,:), q_dot(3,:), q_ddot(3,:)] = Spline.QuinticInterpolation(q_s(3), q_s_d(3), q_s_dd(3), q_e(3), q_e_d(3), q_e_dd(3), t);
        end
    end
end

