% Base class for representing the operational space variables
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef (Abstract) OpSpace < handle
   
    properties
        link                % The link that the coordinate system is attached to
        selection_matrix    % A matrix which projects body velocities into the OPSpace terms
        numOPDofs           % The number of output degrees of freedom
    end
    
    properties (SetAccess = protected)
        id                  % The coordinate system number
        name                % The coordinate system name
    end
        
    methods (Static)
        function o = CreateOPSpace(OPSpaceType)
            switch OPSpaceType
                case OPSpaceType.POSITION
                    o = Position;
                case OPSpaceType.ORIENTATION
                    o = Orientation;
                case OPSpaceType.POSE
                    o = Pose;
                otherwise
                    error('Operational space type is not defined');
            end
            o.type = OPSpaceType;
        end
    end
    
    methods (Abstract)
        % Extraction of the output y given a pose description.
        y = extractOPSpace(obj,x,R)
    end
    
    methods (Abstract,Static)
        o = LoadXmlObj(xmlobj)
    end
    
    methods 
        function T = getSelectionMatrix(obj)
            T = obj.selection_matrix();
        end
    end
    
end

