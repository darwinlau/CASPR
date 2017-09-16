% Used to classify if the DoF is translational or rotational
%
% Author        : Darwin LAU
% Created       : 2017
% Description   : DoFs for the joints are classified either as
% translational or rotational, this is useful for the analysis of CDPRs
classdef DoFType
    enumeration 
        TRANSLATION
        ROTATION
    end
end