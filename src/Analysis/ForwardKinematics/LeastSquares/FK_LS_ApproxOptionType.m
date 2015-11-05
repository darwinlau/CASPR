classdef FK_LS_ApproxOptionType
    % The method used to approximate q_approx
    enumeration 
        USE_PREVIOUS_Q
        FIRST_ORDER_INTEGRATE_QDOT
        FIRST_ORDER_INTEGRATE_PSEUDOINV
    end
end
