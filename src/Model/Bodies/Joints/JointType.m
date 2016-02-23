classdef JointType
    %JointType is the enumeration for type of joint a link has
    enumeration 
        R_X                     % Revolute in X
        R_Y                     % Revolute in Y
        R_Z                     % Revolute in Z
        U_XY                    % Universal with xy-euler
        PLANAR_XY               % Planar in XY plane
        S_EULER_XYZ             % Spherical xyz-euler
        S_FIXED_XYZ             % Spherical xyz-fixed
        SPHERICAL               % Spherical joint using quaternion
        T_XYZ                   % Translational joint XYZ
        SPATIAL                 % T_XYZ + SPHERICAL
        SPATIAL_EULER_XYZ       % T_XYZ + S_EULER_XYZ
    end
end