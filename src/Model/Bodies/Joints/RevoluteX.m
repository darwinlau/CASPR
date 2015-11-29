classdef RevoluteX < Joint
    %RevoluteX Joint definition for a revolute joint in the X-axis
        
    properties (Constant = true)
        numDofs = 1;
        numVars = 1;
        q_default = [0];
        q_dot_default = [0];
        q_ddot_default = [0];
    end
    
    properties (Dependent)
        theta;
        theta_dot;
    end
    
    methods 
        function value = get.theta(obj)
            value = obj.GetTheta(obj.q);
        end
        
        function value = get.theta_dot(obj)
            value = obj.GetTheta(obj.q_dot);
        end
    end
    
    methods (Static)
        function R_pe = RelRotationMatrix(q)
            theta = RevoluteX.GetTheta(q);
            R_pe = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
        end

        function r_rel = RelTranslationVector(~)
            r_rel = [0; 0; 0];
        end
        
        function [S] = RelVelocityMatrix(~)
            S = [0; 0; 0; 1; 0; 0];
        end
        
        function [S_dot] = RelVelocityMatrixDeriv(~, ~)
            S_dot = zeros(6,1);
        end
        
        function [N_j,A] = QuadMatrix(~)
            N_j = 0;
            A = zeros(6,1);
        end
        
        function [q, q_dot, q_ddot] = GenerateTrajectory(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, total_time, time_step)
            t = 0:time_step:total_time;
            [q, q_dot, q_ddot] = Spline.QuinticInterpolation(q_s, q_s_d, q_s_dd, q_e, q_e_d, q_e_dd, t);
        end
        
        % Get variables from the gen coordinates
        function theta = GetTheta(q)
            theta = q(1);
        end
    end
end

