% Base class for uncertainties whose effect can be captured after the
% update
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Base class for uncertainties whose effect can be captured after
%    update. This includes feedback noises.
classdef (Abstract) PostUpdateUncertaintyBase < handle
    methods(Abstract)
%         % Apply uncertainty after the update (currently not used)
%         [update_q,update_q_dot,update_q_ddot] = applyPostUpdateUncertainty(obj,q,q_dot,q_ddot,dt);  
    end
end

