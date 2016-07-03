% Enumeration for the different CDPRs
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :
%    Enumeration for the ModelConfig in order to initialise the appropriate
%    CDPR for simulations. Users must add the enum for their robots added
%    to the library.
classdef TestModelConfigType
    enumeration
        % Testing models
        T_SCDM
        T_MCDM
        T_ACTIVE_PASSIVE_CABLES
    end
end
