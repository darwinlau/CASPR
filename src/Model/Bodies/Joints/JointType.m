classdef JointType
    %JointType is the enumeration for type of joint a link has
    enumeration 
        R_X                     % Revolute in X
        R_Y                     % Revolute in Y
        R_Z                     % Revolute in Z
        PLANAR_XY               % Planar in XY plane
        S_EULER_XYZ             % Spherical xyz-euler
        S_FIXED_XYZ             % Spherical xyz-fixed
    end
end