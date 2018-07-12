% UNITS:
% LEN: METER
% ANGLE: RADIAN
classdef MegaSpoolSpecifications < SpoolSpecificationsBase
    % Parameterize the spool used with Dynamixel
    properties (Constant = true)
        % Spool
        %10.17852 along the bottom of the groove. 
        %  r = 10.17852/27/2/pi, 
        %
        %soconsider the radius r_cable of the cable, 
        %  l = 2*pi*(r+r_cable)*27
        cableLength_full_load = 10.2464;
        numCircles_full_load = 27;
        len_per_circle = MegaSpoolSpecifications.cableLength_full_load/MegaSpoolSpecifications.numCircles_full_load;
        width_per_circle = 0.054/MegaSpoolSpecifications.numCircles_full_load; % the width of one circular coil
        radius = sqrt(MegaSpoolSpecifications.len_per_circle^2-MegaSpoolSpecifications.width_per_circle^2)/2/pi;
        % another way to initialize the spool model
        % width_per_circle = 0.002;
        % radius = 0.03;
        % len_per_circle = sqrt(MegaSpoolSpecifications.width_per_circle^2 + (MegaSpoolSpecifications.radius*2*pi)^2);
        
        % Dynamixel Holder
        lenCoS2Outlet = 0.56; % distance from cetre of the spool to the cable outlet
        cableLengths_full = ones(8,1)*(10.2464+0.5);
    end
    
    
    
    methods
%         function accessories = MegaSpoolSpecifications(radius, width_per_circle, lenCoS2Outlet)
%             accessories.radius = radius;
%             accessories.width_per_circle = width_per_circle;
%             accessories.lenCoS2Outlet = lenCoS2Outlet;
%             accessories.len_per_circle  = sqrt(width_per_circle^2 + (radius*2*pi)^2);
%         end
        function megaaccessories = MegaSpoolSpecifications()
            megaaccessories@SpoolSpecificationsBase();
        end
        
        
    end
end

