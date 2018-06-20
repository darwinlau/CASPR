% Universal joint in the YZ axes
%
% Author        : Benji ZHANG
% Created       : 2017
% Description   :
classdef UniversalYZ < JointBase      
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
        beta
        gamma
        beta_dot
        gamma_dot
    end
    
    methods
        % -------
        % Getters
        % -------
        function value = get.beta(obj)
            value = obj.GetBeta(obj.q);
        end
        function value = get.beta_dot(obj)
            value = obj.GetBeta(obj.q_dot);
        end
        function value = get.gamma(obj)
            value = obj.GetGamma(obj.q);
        end
        function value = get.gamma_dot(obj)
            value = obj.GetGamma(obj.q_dot);
        end
    end
    
    methods (Static)
        % Get the relative rotation matrix
        function R_pe = RelRotationMatrix(q)
            beta    =   UniversalYZ.GetBeta(q);
            gamma   =   UniversalYZ.GetGamma(q);
            R_01    =   RevoluteY.RelRotationMatrix(beta);
            R_12    =   RevoluteZ.RelRotationMatrix(gamma);
            R_pe    =   R_01*R_12;
        end 

        % Get the relative translation vector
        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end 
        
        % Generate the S matrix %% expressed in the {2}
        function S = RelVelocityMatrix(q)
            g = UniversalYZ.GetGamma(q);
            S = [zeros(3,2); sin(g) 0; cos(g) 0; 0 1];    
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(q)
            g = UniversalYZ.GetGamma(q);
            S_grad = MatrixOperations.Initialise([6,2,2], isa(g, 'sym'));
            S_grad(:,:,2)   =   [zeros(3,2); cos(g) 0; -sin(g) 0; 0 0];    
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(q,q_dot)
            g = UniversalYZ.GetGamma(q);
%             c_d = SphericalEulerXYZ.GetGamma(q_dot); % since
%             SphericalEulerXYZ.GetGamma(q) gets q(3)
            g_d = UniversalYZ.GetGamma(q_dot);
            S_dot_grad = MatrixOperations.Initialise([6,2,2], isa(g, 'sym'));
            S_dot_grad(:,:,2)   =   [zeros(3,2); -g_d*sin(g) 0; -g_d*cos(g) 0; 0 0];    
        end   
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(q)
            g = SphericalEulerXYZ.GetBeta(q);
            N_j = [0,0.5*cos(g),0,-0.5*sin(g);0.5*cos(g),0,-0.5*sin(g),0];
            A = [zeros(3,2);1,0;0,1;0,0];
        end
        
        % Get variables from the gen coordinates
        function beta = GetBeta(q)
            beta = q(1);
        end
        function gamma = GetGamma(q)
            gamma = q(2);
        end
        
    end
end