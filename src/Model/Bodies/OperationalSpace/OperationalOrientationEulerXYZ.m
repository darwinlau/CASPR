% Op Orientation class.  Uses EulerXYZ coordinates
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OperationalOrientationEulerXYZ < OperationalSpaceBase
    methods
        % Constructor
        function o = OperationalOrientationEulerXYZ(id,name,link,selection_matrix)
            o.id                =   id;
            o.name              =   name;
            o.link              =   link;
            o.offset            =   zeros(3,1);
            o.numOperationalDofs         =   size(selection_matrix,1);
            % Determine the selection matrix assuming 6 DoF
            temp_selection_matrix = [zeros(3),eye(3)];
            o.selection_matrix  =   temp_selection_matrix(logical(diag(selection_matrix)),:);
        end
        
        % Implementation of the abstract function
        function y = extractOperationalSpace(obj,~,R)
            b = asin(R(1,3));
            g = -atan2(R(1,2), R(1,1));
            a = -atan2(R(2,3), R(3,3));
%             a = round(a, 10);
%             b = round(b, 10);
%             g = round(g, 10);
            y = obj.selection_matrix(:,4:6)*[a;b;g];
        end
    end
    
    methods(Static)
        % Implementation of the load xml obj
        function o = LoadXmlObj(xmlobj)
            id = str2double(char(xmlobj.getAttribute('marker_id')));
            name = char(xmlobj.getAttribute('name'));
            link = str2double(xmlobj.getElementsByTagName('link').item(0).getFirstChild.getData);
            axes_string = char(xmlobj.getElementsByTagName('axes').item(0).getAttribute('active_axes'));
            CASPR_log.Assert(length(axes_string <= 3),'axis string must contain 3 or less characters');
            selection_matrix = zeros(length(axes_string),6);
            for j=1:length(axes_string)
                if(axes_string(j) == 'a')
                    selection_matrix(j,:) = [0,0,0,1,0,0];
                elseif(axes_string(j) == 'b')
                    selection_matrix(j,:) = [0,0,0,0,1,0];
                elseif(axes_string(j) == 'g')
                    selection_matrix(j,:) = [0,0,0,0,0,1];
                else
                    CASPR_log.Print('Unknown character string',CASPRLogLevel.ERROR);
                end
            end
            % obtain selection matrix
            o = OperationalOrientationEulerXYZ(id,name,link,selection_matrix);
        end
    end
end

