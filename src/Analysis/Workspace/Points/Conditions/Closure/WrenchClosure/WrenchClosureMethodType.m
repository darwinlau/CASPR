% An enum class of for wrench closure methods
% Author         : Jonathan EDEN
% Created        : 2015
% Description    : Enum class for different wrench closure methods
classdef WrenchClosureMethodType
    enumeration 
        M_QUAD_PROG
        M_TENSION_FACTOR
        M_UNILATERAL_DEXTERITY
        M_COMBINATORIC_NULL_SPACE
        M_COMBINATORIC_POSITIVE_SPAN
    end
end
