% Object representation of the sphere
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef Hypersphere < handle
        
    properties (SetAccess = protected)
        radius
        origin
    end
    
    methods
        % Constructor for a spherical wrench set.
        function id = Hypersphere(T,r)
            id.origin = T;
            id.radius = r;
        end
    end
end

