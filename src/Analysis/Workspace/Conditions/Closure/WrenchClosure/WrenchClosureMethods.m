classdef WrenchClosureMethods
    enumeration 
        QP
        TF
        UD
        SS
    end
    
    methods (Static)
        function L = workspace_method_list()
            L = {'quad_prog','tension_factor','unilateral_dexterity','semi_singular'};
        end
    end
end