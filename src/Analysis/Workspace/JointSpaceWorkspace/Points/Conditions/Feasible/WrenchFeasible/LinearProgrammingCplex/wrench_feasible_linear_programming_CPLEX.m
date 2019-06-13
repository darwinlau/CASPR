% linear programming (CPLEX) implementation for evaluating WFW.
%
% Author         : Chen SONG
% Created        : 2019
% Description    : Run LP (CPLEX) to check the feasibility of each desired
% wrench inside the given set
function inWorkspace = wrench_feasible_linear_programming_CPLEX(desired_wrench_set,dynamics)
    
    numPoints = size(desired_wrench_set, 2);
    opt_options = cplexoptimset('Display', 'off', 'MaxIter', 100);
    f = zeros(dynamics.numActuatorsActive,1);
    for i = 1:numPoints
        beq = desired_wrench_set(:,i);
        u_lb = dynamics.actuationForcesMin;
        u_ub = dynamics.actuationForcesMax;
        Aeq = -dynamics.L';
        [~, ~, exitflag] = cplexlp(f, [], [], Aeq, beq, u_lb, u_ub, opt_options);
        if exitflag < 0
            inWorkspace = false;
            return
        end
    end
    inWorkspace = true;
    
end