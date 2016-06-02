% Universal joint in the XY axes
%
% Author        : Darwin LAU
% Created       : 2016
% Description   :
classdef UniversalXY < Joint
        
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
        alpha
        beta
        alpha_dot
        beta_dot
    end
    
    methods
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
        function R_pe = RelRotationMatrix(q)
            alpha   =   UniversalXY.GetAlpha(q);
            beta    =   UniversalXY.GetBeta(q);
            R_01    =   RevoluteX.RelRotationMatrix(alpha);
            R_12    =   RevoluteY.RelRotationMatrix(beta);
            R_pe    =   R_01*R_12;
        end

        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end
        
        function S = RelVelocityMatrix(q)
            b = UniversalXY.GetBeta(q);
            S = [zeros(3,2); cos(b) 0; 0 1; sin(b) 0];    
        end
        
        function [S_grad] = RelVelocityMatrixGradient(q)
            b = UniversalXY.GetBeta(q);
            S_grad          =   zeros(6,2,2);
            S_grad(:,:,2)   =   [zeros(3,2); -sin(b) 0; 0 0; cos(b) 0];    
        end
        
        function [S_dot_grad] = RelVelocityMatrixDerivGradient(q,q_dot)
            b = UniversalXY.GetBeta(q);
            b_d = SphericalEulerXYZ.GetBeta(q_dot);
            S_dot_grad          =   zeros(6,2,2);
            S_dot_grad(:,:,2)   =   [zeros(3,2); -b_d*cos(b) 0; 0 0; -b_d*sin(b) 0];    
        end
        
%         function S_dot = RelVelocityMatrixDeriv(q, q_dot)
%             b = SphericalEulerXYZ.GetBeta(q);
%             b_d = SphericalEulerXYZ.GetBeta(q_dot);
%             S_dot = [zeros(3,2); -b_d*sin(b) 0; ...
%                 0 0; ...
%                 b_d*cos(b) 0];
%         end
        
        function [N_j,A] = QuadMatrix(q)
            b = SphericalEulerXYZ.GetBeta(q);
            N_j = [0,-0.5*sin(b),0,0.5*cos(b);-0.5*sin(b),0,0.5*cos(b),0];
            A = [zeros(3,2);1,0;0,0;0,1];
        end
        
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)       
            t = 0:time_step:total_time;
            [q, q_dot, q_ddot] = Spline.QuinticInterpolation(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, t);
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

