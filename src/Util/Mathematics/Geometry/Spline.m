% Library of functions to produce splines
%
% Author        : Darwin LAU
% Created       : 2014
% Description    :
classdef Spline 
    
    properties
    end
    
    methods (Static)
        function [x, x_dot, x_ddot] = QuinticInterpolation(x_s, x_s_dot, x_s_ddot, x_e, x_e_dot, x_e_ddot, t)
            t_s = t(1);
            t_e = t(length(t));
            
            T = [1 t_s t_s^2 t_s^3 t_s^4 t_s^5; ...
                0 1 2*t_s 3*t_s^2 4*t_s^3 5*t_s^4; ...
                0 0 2 6*t_s 12*t_s^2 20*t_s^3; ...
                1 t_e t_e^2 t_e^3 t_e^4 t_e^5; ...
                0 1 2*t_e 3*t_e^2 4*t_e^3 5*t_e^4; ...
                0 0 2 6*t_e 12*t_e^2 20*t_e^3];
            b = [x_s; x_s_dot; x_s_ddot; x_e; x_e_dot; x_e_ddot];
            a = T\b;

            x = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;
            x_dot = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
            x_ddot = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
        end
    end
    
end

