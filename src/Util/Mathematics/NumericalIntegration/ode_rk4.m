% Function for classic Runge-Kutta method for integration
%
% Author        : Darwin LAU
% Created       : 2018
% Description    :
%       Only the function is to do the RK4 method, trying to follow the same
%       inputs as ode45. The input tspan can be two values (start and end)
%       or a set of timesteps and the ode_euler will do integrate on that
%       subdivision
function [t, y] = ode_rk4(odefun, tspan, y0)
    t = tspan;
    y = zeros(length(tspan), length(y0));
    
    y(1,:) = y0;
    
    for i = 2:length(tspan)
        t_curr = tspan(i-1);
        t_next = tspan(i);
        dt = t_next - t_curr;
        y_curr = y(i-1,:)';
        % y_dot returned is a vector
        
        k1 = dt*odefun(t_curr, y_curr);
        k2 = dt*odefun(t_curr + dt/2, y_curr + k1/2);
        k3 = dt*odefun(t_curr + dt/2, y_curr + k2/2);
        k4 = dt*odefun(t_curr + dt, y_curr + k3);
        
        yf = y_curr + (k1 + 2*k2 + 2*k3 + k4)/6;
        % Since yf is a vector, it needs to be transposed to be stored in y
        y(i,:) = yf';        
    end
end

