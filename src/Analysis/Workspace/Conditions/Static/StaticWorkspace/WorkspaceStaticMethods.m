% An enum class of for static workspace methods
% Author         : Jonathan EDEN
% Created        : 2015
% Description    : Enum class for different static workspace methods
classdef WorkspaceStaticMethods
    enumeration 
        QP
        CMa
        CMe
    end
    
    methods (Static)
        % Lists the workspace methods for the GUI
        function L = workspace_method_list()
            L = {'quad_prog','capacity_margin','capability_measure'};
        end
    end
end