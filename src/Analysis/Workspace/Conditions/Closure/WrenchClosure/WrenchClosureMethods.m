classdef WrenchClosureMethods
    enumeration 
        QP
        TF
        UD
        SS
        CNS
        CPS
    end
    
    methods (Static)
        function L = workspace_method_list()
            L = {'quad_prog','tension_factor','unilateral_dexterity','combinatoric_null_space','combinatoric_positive_span'};
        end
    end
end
