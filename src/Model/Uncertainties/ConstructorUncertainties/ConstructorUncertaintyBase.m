% Base class for uncertainties whose effect can be captured at construction 
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Base class for uncertainties whose effect can be captured at
%    construction. This includes inertia and attachment parameters.
classdef (Abstract) ConstructorUncertaintyBase < handle
    methods(Abstract)
        % Apply uncertainty at construction to the system model
        applyConstructorUncertainty(obj,system_model);
    end
end

