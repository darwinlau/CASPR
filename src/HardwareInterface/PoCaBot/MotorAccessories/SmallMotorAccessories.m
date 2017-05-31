% UNITS:
% LEN: METER
% ANGLE: RADIAN
classdef SmallMotorAccessories < MotorAccessoriesBase
    % Parameterize the spool used with Dynamixel
    properties (Constant = true)
        % Spool
        % one way to initialize the spool model
        cableLength_full_load = 2.284;
        numCircles_full_load = 12;
        len_per_circle = SmallMotorAccessories.cableLength_full_load/SmallMotorAccessories.numCircles_full_load;
        width_per_circle = 0.025/SmallMotorAccessories.numCircles_full_load; % the width of one circular coil
        radius = sqrt(SmallMotorAccessories.len_per_circle^2-SmallMotorAccessories.width_per_circle^2)/2/pi;
        % another way to initialize the spool model
        % width_per_circle = 0.002;
        % radius = 0.03;
        % len_per_circle = sqrt(SmallMotorAccessories.width_per_circle^2 + (SmallMotorAccessories.radius*2*pi)^2);
        
        % Dynamixel Holder
        lenCoS2Outlet = 0.110; % distance from cetre of the spool to the cable outlet
    end
    
    
    
    methods
%         function accessories = SmallMotorAccessories(radius, width_per_circle, lenCoS2Outlet)
%             accessories.radius = radius;
%             accessories.width_per_circle = width_per_circle;
%             accessories.lenCoS2Outlet = lenCoS2Outlet;
%             accessories.len_per_circle  = sqrt(width_per_circle^2 + (radius*2*pi)^2);
%         end
        function smallaccessories = SmallMotorAccessories()
            smallaccessories@MotorAccessoriesBase();
        end
        
        
    end
end

