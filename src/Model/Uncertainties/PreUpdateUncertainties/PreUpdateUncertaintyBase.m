% Base class for uncertainties whose effect can be captured before the
% update
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Base class for uncertainties whose effect can be captured before
%    update. This includes disturbances, running obstacles, etc.
classdef (Abstract) PreUpdateUncertaintyBase < handle
    methods(Abstract)
        % Apply uncertainty befoer the update (currently not used)
%         [update_q,update_q_dot,update_q_ddot] = applyPreUpdateUncertainty(obj,q,q_dot,q_ddot,dt);
    end
end

