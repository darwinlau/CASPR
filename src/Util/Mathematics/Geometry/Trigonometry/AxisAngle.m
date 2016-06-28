% Axis angle representation and operations
%
% Author        : Darwin LAU
% Created       : 2014
% Description    :
classdef AxisAngle
    properties
        th
        kx
        ky
        kz
    end

    methods
        % Construct a new axis angle object
        function a = AxisAngle(th, kx, ky, kz)
            if nargin > 0
                a.th = th;
                a.kx = kx;
                a.ky = ky;
                a.kz = kz;
            end
        end
    end

    methods (Static)
        % Convert a quaternion into the axis angle representation
        function a = FromQuaternion(q)
            a = AxisAngle;
            a.th = 2*acos(q.q0);

            if a.th == 0
                a.kx = 0;
                a.ky = 0;
                a.kz = 0;
            else
                a.kx = q.q1/sin(a.th/2);
                a.ky = q.q2/sin(a.th/2);
                a.kz = q.q3/sin(a.th/2);
            end
        end
    end
end
