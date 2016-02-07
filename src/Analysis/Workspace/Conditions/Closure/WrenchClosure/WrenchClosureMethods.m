classdef WrenchClosureMethods
    enumeration 
        QP
        TF
        UD
        SS
        S2015
        L2011
    end
    
    methods (Static)
        function L = workspace_method_list()
            L = {'quad_prog','tension_factor','unilateral_dexterity','semi_singular','Shang_2015','Lim_2011'};
        end
    end
end