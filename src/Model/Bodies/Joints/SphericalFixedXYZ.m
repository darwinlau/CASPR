classdef SphericalFixedXYZ < Joint
    %SphericalXYZ Joint definition for a spherical joint with Euler angles
    %xyz
        
    properties (Constant = true)
        numDofs = 3;
    end
    
    properties (Dependent)
        alpha
        beta
        gamma
        alpha_dot
        beta_dot
        gamma_dot
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
        function value = get.gamma(obj)
            value = obj.GetGamma(obj.q);
        end
        function value = get.gamma_dot(obj)
            value = obj.GetGamma(obj.q_dot);
        end
    end
    
    methods (Static)
        function R_pe = RelRotationMatrix(q)
            alpha = SphericalFixedXYZ.GetAlpha(q);
            beta = SphericalFixedXYZ.GetBeta(q);
            gamma = SphericalFixedXYZ.GetGamma(q);
            R_01 = RevoluteZ.RelRotationMatrix(gamma);
            R_12 = RevoluteY.RelRotationMatrix(beta);
            R_23 = RevoluteX.RelRotationMatrix(alpha);
            R_pe = R_01*R_12*R_23;
        end

        function r_rel = RelTranslation(~)
            r_rel = [0; 0; 0];
        end
        
        function S = RelJointMatrix(q)
            a = SphericalFixedXYZ.GetAlpha(q);
            b = SphericalFixedXYZ.GetBeta(q);
            S = [zeros(3,3); 1 0 -sin(b); 0 cos(a) sin(a)*cos(b); 0 -sin(a) cos(a)*cos(b)];
        end
        
        function S = RelJointMatrixD(q, q_dot)
            a = SphericalFixedXYZ.GetAlpha(q);
            b = SphericalFixedXYZ.GetBeta(q);
            a_d = SphericalFixedXYZ.GetAlpha(q_dot);
            b_d = SphericalFixedXYZ.GetBeta(q_dot);
            S = [zeros(3,3); 0, 0, -b_d*cos(b); 0, -a_d*sin(a), a_d*cos(a)*cos(b)-b_d*sin(a)*sin(b); 0, -a_d*cos(a), -a_d*sin(a)*cos(b)-b_d*cos(a)*sin(b)];
        end
        
        function [N_j,A] = QuadMatrix(q)
            a = SphericalFixedXYZ.GetAlpha(q);
            b = SphericalFixedXYZ.GetBeta(q);
            N_j = [[0,0,0;0,0,-cos(b)/2;0,-cos(b)/2,0],...
                [0,-sin(a)/2,cos(a)*cos(b)/2;-sin(a)/2,0,-sin(a)*sin(b)/2;cos(a)*cos(b)/2,-sin(a)*sin(b)/2,0],...
                [0,-cos(a)/2,-sin(a)*cos(b)/2;-cos(a)/2,0,-cos(a)*sin(b)/2;-sin(a)*cos(b)/2,-cos(a)*sin(b)/2,0]];
            A = [zeros(3);eye(3)];
        end
        
        
        % Get variables from the gen coordinates
        function alpha = GetAlpha(q)
            alpha = q(1);
        end
        function beta = GetBeta(q)
            beta = q(2);
        end
        function gamma = GetGamma(q)
            gamma = q(3);
        end
    end
end

