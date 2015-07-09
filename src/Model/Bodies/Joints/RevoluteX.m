classdef RevoluteX < Joint
    %RevoluteX Joint definition for a revolute joint in the X-axis
        
    properties (Constant = true)
        numDofs = 1;
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

        function r_rel = RelTranslation(~)
            r_rel = [0; 0; 0];
        end
        
        function [S] = RelJointMatrix(~)
            S = [0; 0; 0; 1; 0; 0];
        end
        
        function [S_dot] = RelJointMatrixD(~, ~)
            S_dot = zeros(6,1);
        end
        
        % Get variables from the gen coordinates
        function theta = GetTheta(q)
            theta = q(1);
        end
    end
end

