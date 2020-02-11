% linear programming (CPLEX) implementation for evaluating WFW.
%
% Author         : Chen SONG
% Created        : 2019
% Description    : Run LP (CPLEX) to check the feasibility of each desired
% wrench inside the given set
function inWorkspace = wrench_feasible_linear_programming_CPLEX(desired_wrench_set,dynamics)
    
    opt_options = cplexoptimset('Display', 'off', 'MaxIter', 100);
    f = zeros(dynamics.numActuatorsActive,1);
    u_lb = dynamics.actuationForcesMin;
    u_ub = dynamics.actuationForcesMax;
%     safety_factor = 1e-3;
%     half_gap = (u_ub - u_lb)/2*(1-safety_factor);
%     u_lb = (u_ub + u_lb)/2-half_gap;
%     u_ub = (u_ub + u_lb)/2+half_gap;
    Aeq = [-dynamics.L', dynamics.A];
    for i = 1:desired_wrench_set.numPoints
        beq = desired_wrench_set.points(:,i);
        [~, ~, exitflag] = cplexlp(f, [], [], Aeq, beq, u_lb, u_ub, opt_options);
        if exitflag ~= 1
            inWorkspace = false;
            return
        end
    end
    inWorkspace = true;
    
end