% Base class for uncertainties whose effect can be captured after the
% update
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Base class for uncertainties whose effect can be captured after
%    update. This could be force changes.
classdef (Abstract) PostUpdateUncertaintyBase < handle
    methods(Abstract)
        % Apply uncertainty after the update
        update_force = applyPostUpdateUncertainty(obj,original_force);        
    end
end

