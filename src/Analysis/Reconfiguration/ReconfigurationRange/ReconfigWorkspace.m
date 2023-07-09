% A container class to hold workspace result
%
% Author        : Paul Cheng
% Created       : 2020
% Description    : 
classdef ReconfigWorkspace < handle
    properties (SetAccess = protected)
        model
        q
        reconfigDirection
        reconfigLength
    end
    
    properties
      reconfigRays
    end
    
    properties(Hidden = true)
        
    end
    
    methods
        function rw = ReconfigWorkspace(model,q,reconfigDirection,reconfigLength)
            rw.model = model;
            rw.q = q;
            rw.reconfigDirection = reconfigDirection;
            rw.reconfigLength = reconfigLength;
        end
        
    end
    methods(Access = private)
        
    end
end
