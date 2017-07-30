% Base class for representing the operational space variables
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef (Abstract) OperationalSpaceBase < handle
   
    properties
        link                % The link that the coordinate system is attached to
        selection_matrix    % A matrix which projects body velocities into the Operational Space terms
        numOperationalDofs  % The number of output degrees of freedom
        offset              % The offset of the op_space coordinate in the link frame
    end
    
    properties (SetAccess = protected)
        id                  % The coordinate system number
        name                % The coordinate system name
    end
    
    methods (Abstract)
        % Extraction of the output y given a pose description.
        y = extractOperationalSpace(obj, x, R)
        [y, y_dot, y_ddot] = generateTrajectoryLinearSpline(obj, y_s, y_e, time_vector)
        [y, y_dot, y_ddot] = generateTrajectoryCubicSpline(obj, y_s, y_s_d, y_e, y_e_d, time_vector)
        [y, y_dot, y_ddot] = generateTrajectoryQuinticSpline(obj, y_s, y_s_d, y_s_dd, y_e, y_e_d, y_e_dd, time_vector)
    end
    
    methods (Abstract,Static)
        % Function for loading the xml
        o = LoadXmlObj(xmlobj)
    end
    
    methods 
        % TODO: Remove this
        function T = getSelectionMatrix(obj)
            T = obj.selection_matrix;
        end
    end
    
end

