% Base class for uncertainties associated with pose lock
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Base class for uncertainties associated with pose lock (currently will lock all joints simultaneously)
classdef (Abstract) PoseLockUncertaintyBase < PreUpdateUncertaintyBase
    properties
        model               % Model object
    end
    
    methods
        
        function ipu = PoseLockUncertaintyBase(model) 
            ipu.model           =   model;
        end        
    end
    methods(Abstract)
        % Apply joint lock before the update (indicate whether or not skip FD)
        [is_locked] = applyPoseLock(obj,t);
    end
end

