% Universal joint in the XY axes
%
% Author        : Darwin LAU
% Created       : 2016
% Description   :
classdef UniversalXY < JointBase      
    properties (Constant = true)
        numDofs = 2;
        numVars = 2;
        q_default = [0; 0];
        q_dot_default = [0; 0];
        q_ddot_default = [0; 0];
        q_lb = [-pi; -pi];
        q_ub = [pi; pi];
        q_dofType = [DoFType.ROTATION; DoFType.ROTATION];
    end
    
    properties (Dependent)
        alpha
        beta
        alpha_dot
        beta_dot
    end
    
    methods
        % -------
        % Getters
        % -------
        function value = get.alpha(obj)
            value = obj.GetAlpha(obj.q);
        end
        function value = get.alpha_dot(obj)
            value = obj.GetAlpha(obj.q_dot);
        end
        function value = get.beta(obj)
            value = obj.GetBeta(obj.q);
        end
        function value = get.beta_dot(obj)
            value = obj.GetBeta(obj.q_dot);
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            alpha   =   UniversalXY.GetAlpha(q);
            beta    =   UniversalXY.GetBeta(q);
            R_01    =   RevoluteX.RelRotationMatrix(alpha);
            R_12    =   RevoluteY.RelRotationMatrix(beta);
            R_pe    =   R_01*R_12;
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(q)
            b = UniversalXY.GetBeta(q);
            S = [zeros(3,2); cos(b) 0; 0 1; sin(b) 0];    
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(q)
            b = UniversalXY.GetBeta(q);
            S_grad = MatrixOperations.Initialise([6,2,2], isa(b, 'sym'));
            S_grad(:,:,2)   =   [zeros(3,2); -sin(b) 0; 0 0; cos(b) 0];    
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(q,q_dot)
            b = UniversalXY.GetBeta(q);
            b_d = UniversalXY.GetBeta(q_dot);
            S_dot_grad = MatrixOperations.Initialise([6,2,2], isa(b, 'sym'));
            S_dot_grad(:,:,2)   =   [zeros(3,2); -b_d*cos(b) 0; 0 0; -b_d*sin(b) 0];    
        end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(q)
            b = SphericalEulerXYZ.GetBeta(q);
            N_j = [0,-0.5*sin(b),0,0.5*cos(b);-0.5*sin(b),0,0.5*cos(b),0];
            A = [zeros(3,2);1,0;0,0;0,1];
        end
        
        % Get variables from the gen coordinates
        function alpha = GetAlpha(q)
            alpha = q(1);
        end
        function beta = GetBeta(q)
            beta = q(2);
        end
    end
end

