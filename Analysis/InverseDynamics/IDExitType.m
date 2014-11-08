classdef IDExitType
    %JointType is the enumeration for type of joint a link has
    enumeration 
        NO_ERROR
        LIMIT_REACHED
        INFEASIBLE
        SOLVER_SPECIFIC_ERROR
        OTHER_ERROR
    end
end
