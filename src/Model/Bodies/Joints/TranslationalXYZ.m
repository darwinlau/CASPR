% 3D translation joint in the XYZ axes
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
classdef TranslationalXYZ < JointBase
        
    properties (Constant = true)
        numDofs = 3;
        numVars = 3;
        q_default = [0; 0; 0];
        q_dot_default = [0; 0; 0];
        q_ddot_default = [0; 0; 0];
        q_lb = [-Inf; -Inf; -Inf];
        q_ub = [Inf; Inf; Inf];
        q_dofType = [DoFType.TRANSLATION; DoFType.TRANSLATION; DoFType.TRANSLATION];
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

