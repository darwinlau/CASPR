% The enum class for the quadratic program solver types.
% Author        : Darwin LAU
% Created       : 2015
% Description   : An enum class for qp solvers
classdef ID_QP_SolverType
    enumeration
        MATLAB
        MATLAB_ACTIVE_SET_WARM_START
        MATLAB_INTERIOR_POINT
        OPTITOOLBOX_IPOPT
        OPTITOOLBOX_OOQP
    end
end

