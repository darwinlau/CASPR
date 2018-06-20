% translation joint in the X axis
%
% Author        : Darwin LAU
% Created       : 2017
% Description   :
classdef PrismaticX < JointBase        
    properties (Constant = true)
        numDofs = 1;
        numVars = 1;
        q_default = [0];
        q_dot_default = [0];
        q_ddot_default = [0];
        q_lb = [-Inf];
        q_ub = [Inf];
        q_dofType = [DoFType.TRANSLATION];
    end
    
    properties (Dependent)
        x;
        x_dot;
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
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(~)
            R_pe = eye(3);
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(q)
            x = PrismaticX.GetX(q);
            r_rel = [x; 0; 0];
        end
        
        % Generate the S matrix
        function [S] = RelVelocityMatrix(~)
            S = [1; 0; 0; 0; 0; 0];
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(~)
            S_grad = zeros(6,1,1);
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(~,~)
            S_dot_grad = zeros(6,1,1);
        end
                
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(~)
            N_j = 0;
            A = zeros(6,1);
        end
        
        % Get variables from the gen coordinates
        function theta = GetX(q)
            theta = q(1);
        end
    end
end

