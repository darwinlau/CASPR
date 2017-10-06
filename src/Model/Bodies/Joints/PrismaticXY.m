% Prismatic joints in the XY-plane
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
classdef PrismaticXY < JointBase        
    properties (Constant = true)
        numDofs = 2;
        numVars = 2;
        q_default = [0; 0];
        q_dot_default = [0; 0];
        q_ddot_default = [0; 0];
        q_lb = [-Inf; -Inf];
        q_ub = [Inf; Inf];
        q_dofType = [DoFType.TRANSLATION; DoFType.TRANSLATION];
    end    
    
    properties (Dependent)
        x
        y
        x_dot
        y_dot
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
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(~)
            R_pe = eye(3,3);
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(q)
            x = PrismaticXY.GetX(q);
            y = PrismaticXY.GetY(q);
            r_rel = [x; y; 0];
        end

        % Generate the S matrix
        function S = RelVelocityMatrix(~)
            S = [eye(2);zeros(4,2)];
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(~)
            S_grad = zeros(6,2,2);
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(~,~)
            S_dot_grad = zeros(6,2,2);
        end
                
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            N_j =   zeros(PrismaticXY.numDofs,PrismaticXY.numDofs^2);
            A   =   zeros(6,PrismaticXY.numDofs);
        end
        
        % Get variables from the gen coordinates
        function x = GetX(q)
            x = q(1);
        end
        function y = GetY(q)
            y = q(2);
        end
    end
end

