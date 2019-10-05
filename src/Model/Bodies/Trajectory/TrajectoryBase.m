% Base class for joint and operational space trajectories
%
% Author        : Darwin LAU
% Created       : 2017
% Description   :
classdef (Abstract) TrajectoryBase < handle
    properties
        timeVector      % The time vector for the trajectory
    end
    
    properties (Dependent)
        totalTime 
        timeStep        % The time step for the trajectory
    end
    
    methods        
        function value = get.totalTime(obj)
            value = obj.timeVector(length(obj.timeVector)) - obj.timeVector(1);
        end
        
        % It is assumed that the time-step is uniform for trajectories
        function value = get.timeStep(obj)
            value = obj.timeVector(2) - obj.timeVector(1);
        end
    end
    
    methods (Static, Access=protected)
        function time_abs = get_xml_absolute_tag(xmlobj)
            time_def_str = xmlobj.getAttribute('time_definition');
            time_abs = 0;
            
            if (strcmp(time_def_str, 'relative'))
                time_abs = 0;
            elseif (strcmp(time_def_str, 'absolute'))
                time_abs = 1;
            else
                CASPR_log.Error('Value of attribute ''time_definition'' must either be ''relative'' or ''absolute''');
            end
        end
    end
end