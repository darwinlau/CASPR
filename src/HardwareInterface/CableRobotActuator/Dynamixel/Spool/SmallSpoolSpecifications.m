% UNITS:
% LEN: METER
% ANGLE: RADIAN
classdef SmallSpoolSpecifications < SpoolSpecificationsBase
    % Parameterize the spool used with Dynamixel
    properties (Constant = true)
        % Spool
        % one way to initialize the spool model
        cableLength_full_load = 2.284;
        numCircles_full_load = 12;
        len_per_circle = SmallSpoolSpecifications.cableLength_full_load/SmallSpoolSpecifications.numCircles_full_load;
        width_per_circle = 0.025/SmallSpoolSpecifications.numCircles_full_load; % the width of one circular coil
        radius = sqrt(SmallSpoolSpecifications.len_per_circle^2-SmallSpoolSpecifications.width_per_circle^2)/2/pi;
        % another way to initialize the spool model
        % width_per_circle = 0.002;
        % radius = 0.03;
        % len_per_circle = sqrt(SmallSpoolSpecifications.width_per_circle^2 + (SmallSpoolSpecifications.radius*2*pi)^2);
        
        % Dynamixel Holder
        lenCoS2Outlet = 0.110; % distance from cetre of the spool to the cable outlet
        cableLengths_full = [2;2;2;2;2;2;2;2];
    end
    
    
    
    methods
%         function accessories = SmallSpoolSpecifications(radius, width_per_circle, lenCoS2Outlet)
%             accessories.radius = radius;
%             accessories.width_per_circle = width_per_circle;
%             accessories.lenCoS2Outlet = lenCoS2Outlet;
%             accessories.len_per_circle  = sqrt(width_per_circle^2 + (radius*2*pi)^2);
%         end
        function smallaccessories = SmallSpoolSpecifications()
            smallaccessories@SpoolSpecificationsBase();
        end
        
        
    end
end

