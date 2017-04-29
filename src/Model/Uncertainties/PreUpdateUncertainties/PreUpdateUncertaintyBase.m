% Base class for uncertainties whose effect can be captured before the
% update
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Base class for uncertainties whose effect can be captured before
%    update. This includes kinematic terms and friction
classdef (Abstract) PreUpdateUncertaintyBase < handle
    methods(Abstract)
        % Apply uncertainty after the update
        [update_q,update_q_dot,update_q_ddot,update_w_ext] = applyPreUpdateUncertainty(obj,q,q_dot,q_ddot,w_ext);        
    end
end

