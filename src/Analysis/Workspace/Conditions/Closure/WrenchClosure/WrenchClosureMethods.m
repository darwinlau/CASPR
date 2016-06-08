% An enum class of for wrench closure methods
% Author         : Jonathan EDEN
% Created        : 2015
% Description    : Enum class for different wrench closure methods
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
        % Lists the workspace methods for the GUI
        function L = workspace_method_list()
            L = {'quad_prog','tension_factor','unilateral_dexterity','combinatoric_null_space','combinatoric_positive_span'};
        end
    end
end
