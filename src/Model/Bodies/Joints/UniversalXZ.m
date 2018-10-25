% Universal joint in the XZ axes
%
% Author        : Chen SONG
% Created       : 2018
% Description   :
classdef UniversalXZ < JointBase      
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
        gamma
        alpha_dot
        gamma_dot
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
            alpha   =   UniversalXZ.GetAlpha(q);
            gamma   =   UniversalXZ.GetGamma(q);
            R_01    =   RevoluteX.RelRotationMatrix(alpha);
            R_12    =   RevoluteZ.RelRotationMatrix(gamma);
            R_pe    =   R_01*R_12;
        end

        % Get the relative translation vector
        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end
        
        % Generate the S matrix
        function S = RelVelocityMatrix(q)
            g = UniversalXZ.GetGamma(q);
            S = [zeros(3,2); cos(g) 0; -sin(g) 0; 0 1];
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(q)
            g = UniversalXZ.GetGamma(q);
            S_grad = MatrixOperations.Initialise([6,2,2], isa(g, 'sym'));
            S_grad(:,:,2)   =   [zeros(3,2); -sin(g) 0; -cos(g) 0; 0 0];
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(q,q_dot)
            g = UniversalXZ.GetGamma(q);
            g_d = UniversalXZ.GetGamma(q_dot);
            S_dot_grad = MatrixOperations.Initialise([6,2,2], isa(g, 'sym'));
            S_dot_grad(:,:,2)   =   [zeros(3,2); g_d*cos(g) 0; g_d*sin(g) 0; 0 0];
        end
        
        % Generate the N matrix for the joint
        function [N_j,A] = QuadMatrix(q)
            N_j = [];
            A = [];
            CASPR_log.Error('N matrix calculation currently not supported in Universal XZ joint');
%             b = SphericalEulerXYZ.GetBeta(q);
%             N_j = [0,-0.5*sin(b),0,0.5*cos(b);-0.5*sin(b),0,0.5*cos(b),0];
%             A = [zeros(3,2);1,0;0,0;0,1];
        end
        
        % Get variables from the gen coordinates
        function alpha = GetAlpha(q)
            alpha = q(1);
        end
        function gamma = GetGamma(q)
            gamma = q(2);
        end
    end
end

