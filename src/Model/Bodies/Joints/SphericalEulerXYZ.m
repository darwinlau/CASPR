classdef SphericalEulerXYZ < Joint
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
            alpha = SphericalEulerXYZ.GetAlpha(q);
            beta = SphericalEulerXYZ.GetBeta(q);
            gamma = SphericalEulerXYZ.GetGamma(q);
            R_01 = RevoluteX.RelRotationMatrix(alpha);
            R_12 = RevoluteY.RelRotationMatrix(beta);
            R_23 = RevoluteZ.RelRotationMatrix(gamma);
            R_pe = R_01*R_12*R_23;
        end

        function r_rel = RelTranslation(~)
            r_rel = [0; 0; 0];
        end
        
        function S = RelJointMatrix(q)
            b = SphericalEulerXYZ.GetBeta(q);
            g = SphericalEulerXYZ.GetGamma(q);
            S = [zeros(3,3); cos(b)*cos(g) sin(g) 0; -cos(b)*sin(g) cos(g) 0; sin(b) 0 1];    
        end
        
        function S_dot = RelJointMatrixD(q, q_dot)
            b = SphericalEulerXYZ.GetBeta(q);
            g = SphericalEulerXYZ.GetGamma(q);
            b_d = SphericalEulerXYZ.GetBeta(q_dot);
            g_d = SphericalEulerXYZ.GetGamma(q_dot);
            S_dot = [zeros(3,3); -b_d*sin(b)*cos(g)-g_d*cos(b)*sin(g) g_d*cos(g) 0; ...
                b_d*sin(b)*sin(g)-g_d*cos(b)*cos(g) -g_d*sin(g) 0; ...
                b_d*cos(b) 0 0];
        end
        
        function [N_j,A] = QuadMatrix(q)
            b = SphericalEulerXYZ.GetBeta(q);
            g = SphericalEulerXYZ.GetGamma(q);
            N_j = [0,-0.5*sin(b)*cos(g),-0.5*cos(b)*sin(g),0,0.5*sin(b)*sin(g),-0.5*cos(b)*cos(g),0,0.5*cos(b),0;...
                -0.5*sin(q(2))*cos(g),0,0.5*cos(g),0.5*sin(b)*sin(g),0,-0.5*sin(g),0.5*cos(b),0,0;...
                -0.5*cos(q(2))*sin(g),0.5*cos(g),0,-0.5*cos(b)*cos(g),-0.5*sin(g),0,0,0,0];
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

