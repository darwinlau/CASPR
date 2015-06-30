classdef WrenchSetSphere < handle
    % WrenchSet Class to store information about the wrench set and 
        
    properties (SetAccess = protected)
        r
        T
    end
    
    methods
        function id = WrenchSetSphere(T,r)
            id.T = T;
            id.r = r;
        end
    end
end

