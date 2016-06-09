% Object representation of the wrench-set sphere
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef WrenchSetSphere < handle
        
    properties (SetAccess = protected)
        radius
        origin
    end
    
    methods
        % Constructor for a spherical wrench set.
        function id = WrenchSetSphere(T,r)
            id.origin = T;
            id.radius = r;
        end
    end
end

