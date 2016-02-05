classdef WrenchClosureMethods
    enumeration 
        QP
        TF
        UD
        SS
        S2015
    end
    
    methods (Static)
        function L = workspace_method_list()
            L = {'quad_prog','tension_factor','unilateral_dexterity','semi_singular','Shang_2015'};
        end
    end
end