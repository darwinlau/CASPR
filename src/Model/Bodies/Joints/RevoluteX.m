% Revolute joint in the X axis
%
% Author        : Darwin LAU
% Created       : 2012
% Description   :
classdef RevoluteX < JointBase        
    properties (Constant = true)
        numDofs = 1;
        numVars = 1;
        q_default = [0];
        q_dot_default = [0];
        q_ddot_default = [0];
        q_lb = [-pi];
        q_ub = [pi];
        q_dofType = [DoFType.ROTATION];
    end
    
    properties (Dependent)
        theta;
        theta_dot;
    end
    
    methods 
        % -------
        % Getters
        % -------
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
            theta = RevoluteX.GetTheta(q);
            R_pe = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end
        
        % Generate the S matrix
        function [S] = RelVelocityMatrix(~)
            S = [0; 0; 0; 1; 0; 0];
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
        function theta = GetTheta(q)
            theta = q(1);
        end
    end
end

