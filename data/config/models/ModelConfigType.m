% Enumeration for the different CDPRs
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :
%    Enumeration for the ModelConfig in order to initialise the appropriate
%    CDPR for simulations. Users must add the enum for their robots added
%    to the library.
classdef ModelConfigType
    enumeration
        % Planar manipulators
        M_2DOF_VSD
        M_KNTU_PLANAR
        M_SIMPLE_PLANAR_XY
        M_PASSIVE_SPRINGS_PLANAR
        % Spatial manipulators
        M_ACROBOT
		M_COGIRO
        M_IPANEMA_1
        M_IPANEMA_2
        M_NIST_ROBOCRANE
        M_SEGESTA
        M_FAST
        M_SIMPLE_SPATIAL
        % Spherical manipulators
        M_MYOROB_SHOULDER
        M_SIMPLE_SPHERICAL
        % MCDMs
        M_2R_PLANAR_XZ
        M_NECK_8S
        M_CAREX
    end
end
