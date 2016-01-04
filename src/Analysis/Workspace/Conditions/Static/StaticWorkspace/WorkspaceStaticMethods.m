classdef WorkspaceStaticMethods
    enumeration 
        QP
        CMa
        CMe
    end
    
    methods (Static)
        function L = workspace_method_list()
            L = {'quad_prog','capacity_margin','capability_measure'};
        end
    end
end