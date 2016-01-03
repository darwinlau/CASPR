classdef IDSolverExitType
    %JointType is the enumeration for type of joint a link has
    enumeration 
        NO_ERROR
        INFEASIBLE
        ITERATION_LIMIT_REACHED
        SOLVER_SPECIFIC_ERROR
        OTHER_ERROR
    end
end
