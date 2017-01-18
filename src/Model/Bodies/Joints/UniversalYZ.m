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
            c = UniversalYZ.GetGamma(q);
            S = [zeros(3,2); sin(c) 0; cos(c) 0; 0 1];    
        end
        
        % Generate the S gradient tensor
        function [S_grad] = RelVelocityMatrixGradient(q)
            c = UniversalYZ.GetGamma(q);
            S_grad          =   zeros(6,2,2);
            S_grad(:,:,2)   =   [zeros(3,2); cos(c) 0; -sin(c) 0; 0 0];    
        end
        
        % Generate the \dot{S} gradient tensor
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(q,q_dot)
            c = UniversalYZ.GetGamma(q);
%             c_d = SphericalEulerXYZ.GetGamma(q_dot); % since
%             SphericalEulerXYZ.GetGamma(q) gets q(3)
            c_d = UniversalYZ.GetGamma(q_dot);
            S_dot_grad          =   zeros(6,2,2);
            S_dot_grad(:,:,2)   =   [zeros(3,2); -c_d*sin(c) 0; -c_d*cos(c) 0; 0 0];    
        end   
        
%         function S_dot = RelVelocityMatrixDeriv(q, q_dot)
%             b = SphericalEulerXYZ.GetBeta(q);
%             b_d = SphericalEulerXYZ.GetBeta(q_dot);
%             S_dot = [zeros(3,2); -b_d*sin(b) 0; ...
%                 0 0; ...
%                 b_d*cos(b) 0];
%         end
        
        %%%%%%%[No N_j and A matrixes]%%%%%%%%%%%%%%
        
        % Generate the trajectory for this joint
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)       
            t = 0:time_step:total_time;
            [q, q_dot, q_ddot] = Spline.QuinticInterpolation(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, t);
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