% Function for euler method for crude but faster integration
%
% Author        : Darwin LAU
% Created       : 2018
% Description    :
%       Only the function is to do the Euler method, trying to follow the same
%       inputs as ode45. The input tspan can be two values (start and end)
%       or a set of timesteps and the ode_euler will do integrate on that
%       subdivision
function [t, y] = ode_euler(odefun, tspan, y0)
    t = tspan;
    y = zeros(length(tspan), length(y0));
    
    y(1,:) = y0;
    
    for i = 2:length(tspan)
        t0 = tspan(i-1);
        tf = tspan(i);
        dt = tf - t0;
        % y_dot returned is a vector
        y_dot = odefun(t0, y(i-1,:)');
        yf = y(i-1,:)' + dt * y_dot;
        % Since yf is a vector, it needs to be transposed to be stored in y
        y(i,:) = yf';        
    end
end

