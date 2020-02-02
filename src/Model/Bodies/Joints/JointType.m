% Enum for the type of joint
%
% Author        : Darwin LAU
% Created       : 2015
% Description   :
classdef JointType
    enumeration 
        R_X                     % Revolute in X
        R_Y                     % Revolute in Y
        R_Z                     % Revolute in Z
        R_AXIS                  % Revolute in defined axis of rotation
        U_XY                    % Universal with xy-euler
        U_YZ                    % Universal with yz-euler
        U_XZ                    % Universal with xz-euler
        P_X                     % Translational X
        P_XY                    % Translational joint in XY plane
        P_XZ                    % Translational joint in XZ plane
        P_AXIS                  % Translational joint in defined axis
        PLANAR_XY               % Planar in XY plane
        PLANAR_YZ               % Planar in YZ plane
        PLANAR_XZ               % Planar in XZ plane %%
        S_EULER_XYZ             % Spherical xyz-euler
        S_FIXED_XYZ             % Spherical xyz-fixed
%         S_QUATERNION            % Spherical joint using quaternion
        T_XYZ                   % Translational joint XYZ
%         SPATIAL_QUATERNION      % T_XYZ + SPHERICAL
        SPATIAL_EULER_XYZ       % T_XYZ + S_EULER_XYZ
    end
end