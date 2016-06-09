% Op Pose class.  Uses EulerXYZ coordinates
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OpPoseEulerXYZ < OpSpace
    methods
        % Constructor
        function o = OpPoseEulerXYZ(id,name,link,offset,selection_matrix)
            o.id                =   id;
            o.name              =   name;
            o.link              =   link;
            o.offset            =   offset;
            o.numOPDofs         =   sum(diag(selection_matrix));
            % Determine the selection matrix assuming 6 DoF
            temp_selection_matrix = eye(6);
            o.selection_matrix  =   temp_selection_matrix(logical(diag(selection_matrix)),:);
        end
        
        % Implementation of the abstract function
        function y = extractOpSpace(obj,x,R)
            y_x = obj.selection_matrix(:,1:3)*R*x;
            b   = asin(R(1,3));
            g   = -atan2(R(1,2), R(1,1));
            a   = -atan2(R(2,3), R(3,3));
%             a   = roundn(a, -10);
%             b   = roundn(b, -10);
%             g   = roundn(g, -10);
            y_r = obj.selection_matrix(:,4:6)*[a;b;g];
            y   = [y_x;y_r];
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
            s_a = str2double(selectionObj.getElementsByTagName('sa').item(0).getFirstChild.getData);
            s_b = str2double(selectionObj.getElementsByTagName('sb').item(0).getFirstChild.getData);
            s_g = str2double(selectionObj.getElementsByTagName('sg').item(0).getFirstChild.getData);
            selection_matrix = diag([s_x,s_y,s_z,s_a,s_b,s_g]);
            % obtain selection matrix
            o = OpPoseEulerXYZ(id,name,link,link_offset,selection_matrix);
        end
    end
end

