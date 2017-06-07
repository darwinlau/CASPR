% Op Orientation class.  Uses EulerXYZ coordinates
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   :
classdef OperationalOrientationEulerXYZ < OperationalSpace
    methods
        % Constructor
        function o = OperationalOrientationEulerXYZ(id,name,link,selection_matrix)
            o.id                =   id;
            o.name              =   name;
            o.link              =   link;
            o.offset            =   zeros(3,1);
            o.numOperationalDofs         =   sum(diag(selection_matrix));
            % Determine the selection matrix assuming 6 DoF
            temp_selection_matrix = [zeros(3),eye(3)];
            o.selection_matrix  =   temp_selection_matrix(logical(diag(selection_matrix)),:);
        end
        
        % Implementation of the abstract function
        function y = extractOperationalSpace(obj,~,R)
            b = asin(R(1,3));
            g = -atan2(R(1,2), R(1,1));
            a = -atan2(R(2,3), R(3,3));
            y = obj.selection_matrix(:,4:6)*[a;b;g];
        end
    end
    
    methods(Static)
        % Implementation of the load xml obj
        function o = LoadXmlObj(xmlobj)
            id = str2double(char(xmlobj.getAttribute('num')));
            name = char(xmlobj.getAttribute('name'));
            link = str2double(xmlobj.getElementsByTagName('link').item(0).getFirstChild.getData);
            selectionObj = xmlobj.getElementsByTagName('selection_matrix').item(0);
            s_a = str2double(selectionObj.getElementsByTagName('sa').item(0).getFirstChild.getData);
            s_b = str2double(selectionObj.getElementsByTagName('sb').item(0).getFirstChild.getData);
            s_g = str2double(selectionObj.getElementsByTagName('sg').item(0).getFirstChild.getData);
            selection_matrix = diag([s_a,s_b,s_g]);
            % obtain selection matrix
            o = OperationalOrientationEulerXYZ(id,name,link,selection_matrix);
        end
    end
end

