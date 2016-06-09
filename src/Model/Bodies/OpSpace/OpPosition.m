% Represents the operational space position for a particular link
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OpPosition < OpSpace   
    
    methods
        % Constructor for the class
        function o = OpPosition(id,name,link,offset,selection_matrix)
            o.id                =   id;
            o.name              =   name;
            o.link              =   link;
            o.offset            =   offset;
            o.numOPDofs     =   sum(diag(selection_matrix));
            % Determine the selection matrix assuming 6 DoF
            temp_selection_matrix = [eye(3),zeros(3)];
            o.selection_matrix  =   temp_selection_matrix(logical(diag(selection_matrix)),:);
        end
        
        % Implementation of the abstract function
        function y = extractOpSpace(obj,x,R)
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
            selectionObj = xmlobj.getElementsByTagName('selection_matrix').item(0);
            s_x = str2double(selectionObj.getElementsByTagName('sx').item(0).getFirstChild.getData);
            s_y = str2double(selectionObj.getElementsByTagName('sy').item(0).getFirstChild.getData);
            s_z = str2double(selectionObj.getElementsByTagName('sz').item(0).getFirstChild.getData);
            selection_matrix = diag([s_x,s_y,s_z]);
            % obtain selection matrix
            o = OpPosition(id,name,link,link_offset,selection_matrix);
        end
    end
end

