classdef Grid < handle
    % Grid Grid class to allow for both uniform and other grips
        
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

