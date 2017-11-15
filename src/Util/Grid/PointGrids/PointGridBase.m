% Base class for the representation and functions on grids
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :
classdef PointGridBase < handle
        
    properties (SetAccess = protected)
        n_dimensions        % The number of dimensions for the system
        n_points            % The number of points in the grid
    end
    
    methods (Abstract)
        % Get a grid point given the grid index
        f = getGridPoint(obj,index)
    end
    
    methods
        % TODO: Make the setters consistent
        
        % -------
        % Setters
        % -------
        function setNDimensions(obj,n_dimensions)
            obj.n_dimensions = n_dimensions;
        end
        
        function setNPoints(obj,n_points)
            obj.n_points = n_points;
        end
    end
end

