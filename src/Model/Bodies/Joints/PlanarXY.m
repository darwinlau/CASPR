% Planar joint in the XY-plane
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
classdef PlanarXY < JointBase        
    properties (Constant = true)
        numDofs = 3;
        numVars = 3;
        q_default = [0; 0; 0];
        q_dot_default = [0; 0; 0];
        q_ddot_default = [0; 0; 0];
        q_lb = [-Inf; -Inf; -pi];
        q_ub = [Inf; Inf; pi];
        q_dofType = [DoFType.TRANSLATION; DoFType.TRANSLATION; DoFType.ROTATION];
    end    
    
    properties (Dependent)
        x
        y
        theta
        x_dot
        y_dot
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
        function value = get.y(obj)
            value = obj.GetY(obj.q);
        end
        function value = get.y_dot(obj)
            value = obj.GetY(obj.q_dot);
        end
        function value = get.theta(obj)
            value = obj.GetTheta(obj.q);
        end
        function value = get.theta_dot(obj)
            value = obj.GetTheta(obj.q_dot);
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            theta = PlanarXY.GetTheta(q);
            R_pe = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(q)
            x = PlanarXY.GetX(q);
            y = PlanarXY.GetY(q);
            r_rel = [x; y; 0];
        end

        % Generate the S matrix
        function S = RelVelocityMatrix(~)
            S = [1 0 0; 0 1 0; 0 0 0; 0 0 0; 0 0 0; 0 0 1];
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
            N_j = zeros(PlanarXY.numDofs,PlanarXY.numDofs^2);
            A = zeros(6,PlanarXY.numDofs);
        end
        
        % Get variables from the gen coordinates
        function x = GetX(q)
            x = q(1);
        end
        function y = GetY(q)
            y = q(2);
        end
        function theta = GetTheta(q)
            theta = q(3);
        end
        
        % Should this be a separate function
        function theta_d = GetThetaDot(q_dot)
            theta_d = q_dot(3);
        end
    end
end

