classdef PlanarXY < Joint
    %RevoluteX Joint definition for a revolute joint in the X-axis
        
    properties (Constant = true)
        numDofs = 3;
    end    
    
    properties (Dependent)
        x
        y
        theta
        x_dot
        y_dot
        theta_dot
    end
    
    methods 
        function value = get.x(obj)
            value = obj.GetX(obj.q);
        end
        function value = get.x_dot(obj)
            value = obj.GetX(obj.q_dot);
        end
        function value = get.y(obj)
            value = obj.GetY(obj.q);
        end
        function value = get.y_dot(obj)
            value = obj.GetY(obj.q_dot);
        end
        function value = get.theta(obj)
            value = obj.GetTheta(obj.q);
        end
        function value = get.theta_dot(obj)
            value = obj.GetTheta(obj.q_dot);
        end
    end
    
    methods (Static)
        function R_pe = RelRotationMatrix(q)
            theta = PlanarXY.GetTheta(q);
            R_pe = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
        end

        function r_rel = RelTranslation(q)
            x = PlanarXY.GetX(q);
            y = PlanarXY.GetY(q);
            r_rel = [x; y; 0];
        end
        
        function S = RelJointMatrix(q)
            th = PlanarXY.GetTheta(q);
            S = [cos(th) sin(th) 0; -sin(th) cos(th) 0; 0 0 0; 0 0 0; 0 0 0; 0 0 1];
        end
        
        function S_dot = RelJointMatrixD(q, q_dot)
            th = PlanarXY.GetTheta(q);
            th_d = PlanarXY.GetThetaDot(q_dot);
            S_dot = [-th_d*sin(th) th_d*cos(th) 0; -th_d*cos(th) -th_d*sin(th) 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0];
        end
        
        function [N_j,A] = QuadMatrix(q)
            th = PlanarXY.GetTheta(q);
            N_j = [[0,0,-sin(th)/2;0,0,cos(th)/2;-sin(th)/2,cos(th)/2,0],...
                [0,0,-cos(th)/2;0,0,-sin(th)/2;-cos(th)/2,-sin(th)/2,0],...
                [0,0,0;0,0,0;0,0,0]];
            A = [eye(3);zeros(3)];
        end
        
        % Get variables from the gen coordinates
        function x = GetX(q)
            x = q(1);
        end
        function y = GetY(q)
            y = q(2);
        end
        function theta = GetTheta(q)
            theta = q(3);
        end
        function theta_d = GetThetaDot(q_dot)
            theta_d = q_dot(3);
        end
    end
end

