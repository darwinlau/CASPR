% UNITS:
% LEN: METER
% ANGLE: RADIAN
classdef LargeMotorAccessories < MotorAccessoriesBase
    % Parameterize the spool used with Dynamixel
    properties (Constant = true)
        % Spool
        % one way to initialize the spool model
        cableLength_full_load = 2.284;%2.284;
        numCircles_full_load = 12;%12;
        len_per_circle = LargeMotorAccessories.cableLength_full_load/LargeMotorAccessories.numCircles_full_load;
        width_per_circle = 0.025/LargeMotorAccessories.numCircles_full_load; % the width of one circular coil
        radius = sqrt(LargeMotorAccessories.len_per_circle^2-LargeMotorAccessories.width_per_circle^2)/2/pi;
        % another way to initialize the spool model
        % width_per_circle = 0.002;
        % radius = 0.03;
        % len_per_circle = sqrt(LargeMotorAccessories.width_per_circle^2 + (LargeMotorAccessories.radius*2*pi)^2);
        
        % Dynamixel Holder
        lenCoS2Outlet = 0.210; % distance from cetre of the spool to the cable outlet
        cableLengths_full = [6.618; 4.800; 6.632;4.800;6.632;5.545;6.618;5.545];
    end
    
    
    
    methods
%         function accessories = LargeMotorAccessories(radius, width_per_circle, lenCoS2Outlet)
%             accessories.radius = radius;
%             accessories.width_per_circle = width_per_circle;
%             accessories.lenCoS2Outlet = lenCoS2Outlet;
%             accessories.len_per_circle  = sqrt(width_per_circle^2 + (radius*2*pi)^2);
%         end
        function largeaccessories = LargeMotorAccessories()
            largeaccessories@MotorAccessoriesBase();
        end
        
        
    end
end

