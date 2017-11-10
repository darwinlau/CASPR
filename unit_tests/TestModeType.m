% Enum for the type of unit testing to be performed
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description   :
classdef TestModeType
    enumeration 
        DEFAULT             % Test model config, model and analysis
        MODEL_CONFIG
        MODEL_COMPONENTS
        ANALYSIS
        SCRIPTS
        ALL
    end
end