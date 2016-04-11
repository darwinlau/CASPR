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
        offset              % The offset of the op_space coordinate in the link frame
    end
    
    properties (SetAccess = protected)
        id                  % The coordinate system number
        name                % The coordinate system name
    end
    
    methods (Abstract)
        % Extraction of the output y given a pose description.
        y = extractOpSpace(obj,x,R)
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

