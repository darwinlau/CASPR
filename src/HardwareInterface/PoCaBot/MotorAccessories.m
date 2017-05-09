% UNITS:
% LEN: METER
% ANGLE: RADIAN
classdef MotorAccessories < handle
    % Parameterize the spool used with Dynamixel
    properties (Access = private, Constant = true)
        % Spool
        % one way to initialize the spool model
        cableLength_full_load = 2;
        numCircles_full_load = 10;
        len_per_circle = MotorAccessories.cableLength_full_load/MotorAccessories.numCircles_full_load;
        width_per_circle = 0.002; % the width of one circular coil
        radius = sqrt(MotorAccessories.len_per_circle^2-MotorAccessories.width_per_circle^2)/2/pi;
        % another way to initialize the spool model
        % width_per_circle = 0.002;
        % radius = 0.03;
        % len_per_circle = sqrt(MotorAccessories.width_per_circle^2 + (MotorAccessories.radius*2*pi)^2);
        
        % Dynamixel Holder
        lenCoS2Outlet = 0.04; % distance from cetre of the spool to the cable outlet
    end
    
    properties
        % When the cable is loosing up, coil_position is increasing. When
        % this property is zero, the loosed segment of the cable in the
        % holder reaches minimal value.
        % coil_position % unused here
        % cableLength_in_accessories_init
        
        relTheta_init
        coil_position_init
    end
    
    methods
%         function accessories = MotorAccessories(radius, width_per_circle, lenCoS2Outlet)
%             accessories.radius = radius;
%             accessories.width_per_circle = width_per_circle;
%             accessories.lenCoS2Outlet = lenCoS2Outlet;
%             accessories.len_per_circle  = sqrt(width_per_circle^2 + (radius*2*pi)^2);
%         end
        function accessories = MotorAccessories()
        end
        
        % deltaAngle UNIT: RADIAN
        % deltaLength is relative to the initial cable length when the
        % coil_position = coil_position_init;
        function [deltaAngle] = getDeltaAngle(obj, deltaLength)
            % set
            b = obj.len_per_circle/2/pi;
            e = obj.coil_position_init;
            a = obj.width_per_circle/2/pi;
            d = sqrt(obj.lenCoS2Outlet^2+obj.radius^2);
            y = deltaLength;
            
            % next solve the equations:
            % solve(y == b*x-(sqrt(d^2+(e+a*x)^2)-sqrt(d^2+e^2)),x)
            % x here means the delta angle
            deltaAngle = -((a^2*e^2 + b^2*d^2 + b^2*e^2 + a^2*y^2 - 2*a^2*y*(d^2 + e^2)^(1/2) + 2*a*b*e*y - 2*a*b*e*(d^2 + e^2)^(1/2))^(1/2) - b*(d^2 + e^2)^(1/2) + a*e + b*y)/(a^2 - b^2);
            CASPR_log.Assert(deltaAngle < obj.relTheta_init,'Cable has reached its limit! No loosing!');
        end
        
        function [deltaLength] = getDeltaLength(obj, delta_angle)
            deltaLength = obj.len_per_circle*delta_angle/2/pi- (sqrt(obj.lenCoS2Outlet^2+obj.radius^2 + (obj.coil_position_init + delta_angle * obj.width_per_circle/2/pi)^2) - sqrt(obj.lenCoS2Outlet^2+obj.radius^2 + obj.coil_position_init^2));
        end
        
        function setInitState(obj, len_in_accessories)
            % set
            b = obj.len_per_circle/2/pi;
            m = obj.numCircles_full_load*obj.width_per_circle/2;
            a = obj.width_per_circle/2/pi;
            d = sqrt(obj.lenCoS2Outlet^2+obj.radius^2);
            y = len_in_accessories;
            
            CASPR_log.Assert(len_in_accessories>=sqrt(d^2+m^2),'Input Argument len_in_accessories should be non-negative');
            
            % next solve the equations:
            % solve(y == b*x+sqrt(d^2+(m-a*x)^2),b*x<y,x)
            % x here means the delta angle
            x = (a*m - b*y + (- a^2*d^2 + a^2*y^2 - 2*a*b*m*y + b^2*d^2 + b^2*m^2)^(1/2))/(a^2 - b^2);
            obj.relTheta_init = x;
            obj.coil_position_init = m - x*a;
        end
    end
end

