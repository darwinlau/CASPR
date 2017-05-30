% Library of functions to produce splines
%
% Author        : Darwin LAU
% Created       : 2014
% Description    :
classdef Spline
    methods (Static)
        % Linear interpolation (only specifying the start and end point)
        function [x, x_dot, x_ddot] = LinearInterpolation(x_s, x_e, time_vector)
            t_s = time_vector(1);
            t_e = time_vector(length(time_vector));
            T = [1 t_s; ...
                1 t_e];
            b = [x_s; x_e];
            a = T\b;
            x = a(1) + a(2)*time_vector;
            x_dot = a(2)*ones(size(time_vector));
            x_ddot = zeros(size(time_vector));
        end
        
        % Cubic interpolation (specifying start and end point/velocity)
        function [x, x_dot, x_ddot] = CubicInterpolation(x_s, x_s_dot, x_e, x_e_dot, time_vector)
            t_s = time_vector(1);
            t_e = time_vector(length(time_vector));
            T = [1 t_s t_s^2 t_s^3; ...
                0 1 2*t_s 3*t_s^2;
                1 t_e t_e^2 t_e^3; ...
                0 1 2*t_e 3*t_e^2];
            b = [x_s; x_s_dot; x_e; x_e_dot];
            a = T\b;
            x = a(1) + a(2)*time_vector + a(3)*time_vector.^2 + a(4)*time_vector.^3;
            x_dot = a(2) + 2*a(3)*time_vector + 3*a(4)*time_vector.^2;
            x_ddot = 2*a(3)*ones(size(time_vector)) + 6*a(4)*time_vector;
        end
        
        % Quintic interpolation (specifying start and end
        % point/velocity/acceleration)
        function [x, x_dot, x_ddot] = QuinticInterpolation(x_s, x_s_dot, x_s_ddot, x_e, x_e_dot, x_e_ddot, time_vector)
            t_s = time_vector(1);
            t_e = time_vector(length(time_vector));
            T = [1 t_s t_s^2 t_s^3 t_s^4 t_s^5; ...
                0 1 2*t_s 3*t_s^2 4*t_s^3 5*t_s^4; ...
                0 0 2 6*t_s 12*t_s^2 20*t_s^3; ...
                1 t_e t_e^2 t_e^3 t_e^4 t_e^5; ...
                0 1 2*t_e 3*t_e^2 4*t_e^3 5*t_e^4; ...
                0 0 2 6*t_e 12*t_e^2 20*t_e^3];
            b = [x_s; x_s_dot; x_s_ddot; x_e; x_e_dot; x_e_ddot];
            a = T\b;
            x = a(1) + a(2)*time_vector + a(3)*time_vector.^2 + a(4)*time_vector.^3 + a(5)*time_vector.^4 + a(6)*time_vector.^5;
            x_dot = a(2) + 2*a(3)*time_vector + 3*a(4)*time_vector.^2 + 4*a(5)*time_vector.^3 + 5*a(6)*time_vector.^4;
            x_ddot = 2*a(3) + 6*a(4)*time_vector + 12*a(5)*time_vector.^2 + 20*a(6)*time_vector.^3;
        end
        
        % Parabolic blend (quadratic start and end for zero velocities with
        % linear section in the middle)
        function [x, x_dot, x_ddot] = ParabolicBlend(x_s, x_e, time_vector, time_blend)
            t_s = time_vector(1);
            t_e = time_vector(length(time_vector));
            t_b = t_s + time_blend;
            
            x_accel = (x_s - x_e)/(time_blend^2 - (t_e - t_s)*time_blend);
            x_b = x_s + x_accel*time_blend^2/2;
            v_lin = ((x_s+x_e)/2 - x_b)/((t_e-t_s)/2 - time_blend);
            
            x = zeros(1, length(time_vector));
            x_dot = zeros(1, length(time_vector));
            x_ddot = zeros(1, length(time_vector));
            
            for t_ind = 1:length(time_vector)
                t = time_vector(t_ind);
                if (t < t_b)
                    x(t_ind) = x_s + x_accel*(t-t_s)^2/2;
                    x_dot(t_ind) = x_accel*(t-t_s);
                    x_ddot(t_ind) = x_accel;
                elseif (t < t_e - time_blend)
                    x(t_ind) = x_b + v_lin * (t-t_b);
                    x_dot(t_ind) = v_lin;
                    x_ddot(t_ind) = 0;
                else
                    x(t_ind) = -x_accel*(t-t_e)^2/2 + x_e;
                    x_dot(t_ind) = -x_accel*(t-t_e);
                    x_ddot(t_ind) = -x_accel;
                end
            end
        end
    end
end
