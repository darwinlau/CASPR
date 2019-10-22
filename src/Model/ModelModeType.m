% Enum for the type of model updating to be performed
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description   :
classdef ModelModeType
    enumeration 
        DEFAULT         % Standard model updating
        SYMBOLIC        % Symbolic model
        COMPILED        % Model is to use compiled functions
    end
end