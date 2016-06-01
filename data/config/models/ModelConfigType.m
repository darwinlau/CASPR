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
        M_SIMPLE_PLANAR_XY
        M_2DOF_VSD
        M_SIMPLE_SPHERICAL
        M_SIMPLE_SPATIAL
        M_NECK_8S
        M_2R_PLANAR_XZ
        M_ACROBOT
        M_IPANEMA_2
        M_MYOROB_SHOULDER
        M_NIST_ROBOCRANE
		M_COGIRO
        M_CAREX
        M_KNTU_PLANAR
    end
end