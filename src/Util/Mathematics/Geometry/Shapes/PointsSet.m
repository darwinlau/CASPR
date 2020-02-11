% Object representation of a set of points
%
% Author        : Darwin LAU
% Created       : 2020
% Description    :
classdef PointsSet < handle
        
    properties (SetAccess = protected)
        points = []                 % All points passed in to create the convex polytope
        numDofs = 0                 % Number of DoFs of the convex polytope
    end
    
    properties (Dependent)
        numPoints
    end
    
    methods        
        % Points set P are passed in as a matrix where each column
        % represent one point
        function ps = PointsSet(P)
            ps.points = P;
            % Dimension of point is the number of rows
            ps.numDofs = size(P, 1);
        end
    end
    
    % Getters and setters
    methods
        function val = get.numPoints(obj)
            val = size(obj.points, 2);
        end
    end
end