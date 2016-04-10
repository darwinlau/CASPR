% Object representation of the wrench-set sphere
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef WrenchSetSphere < handle
        
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

