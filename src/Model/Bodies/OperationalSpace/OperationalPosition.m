% Represents the operational space position for a particular link
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OperationalPosition < OperationalSpaceBase
    
    methods
        % Constructor for the class
        function o = OperationalPosition(id,name,link,offset,selection_matrix)
            o.id                =   id;
            o.name              =   name;
            o.link              =   link;
            o.offset            =   offset;
            o.numOperationalDofs     =   size(selection_matrix,1);
            % Determine the selection matrix assuming 6 DoF            
            o.selection_matrix  =   selection_matrix;
        end
        
        % Implementation of the abstract function
        function y = extractOperationalSpace(obj,x,R)
            y = obj.selection_matrix(:,1:3)*R*x;
        end
    end
    
    methods(Static)
        % Implementation of the load xml obj
        function o = LoadXmlObj(xmlobj)
            id = str2double(char(xmlobj.getAttribute('num')));
            name = char(xmlobj.getAttribute('name'));
            link = str2double(xmlobj.getElementsByTagName('link').item(0).getFirstChild.getData);
            link_offset = XmlOperations.StringToVector3(char(xmlobj.getElementsByTagName('offset').item(0).getFirstChild.getData));
            axes_string = char(xmlobj.getElementsByTagName('axes').item(0).getAttribute('active_axes'));
            CASPR_log.Assert(length(axes_string <= 3),'axis string must contain 3 or less characters');
            selection_matrix = zeros(length(axes_string),6);
            for j=1:length(axes_string)
                if(axes_string(j) == 'x')
                    selection_matrix(j,:) = [1,0,0,0,0,0];
                elseif(axes_string(j) == 'y')
                    selection_matrix(j,:) = [0,1,0,0,0,0];
                elseif(axes_string(j) == 'z')
                    selection_matrix(j,:) = [0,0,1,0,0,0];
                else
                    CASPR_log.Print('Unknown character string',CASPRLogLevel.ERROR);
                end
            end
            CASPR_log.Assert(rank(selection_matrix) == length(axes_string),'Character string should not repeat');
            o = OperationalPosition(id,name,link,link_offset,selection_matrix);
        end
    end
end

