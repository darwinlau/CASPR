% Base class for the representation and functions on grids
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef Grid < handle
        
    properties (SetAccess = protected)
        n_dimensions
        n_points
    end
    
    methods (Abstract)
        f = getGridPoint(obj,index)
    end
    
    methods
        function setNDimensions(obj,n_dimensions)
            obj.n_dimensions = n_dimensions;
        end
        
        function setNPoints(obj,n_points)
            obj.n_points = n_points;
        end
    end
end

