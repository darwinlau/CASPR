% linear programming (Matlab) implementation for evaluating WFW.
%
% Author         : Chen SONG
% Created        : 2019
% Description    : Run LP (Matlab) to check the feasibility of each desired
% wrench inside the given set
function inWorkspace = wrench_feasible_linear_programming_MATLAB(desired_wrench_set,dynamics)
    
    opt_options = optimoptions('linprog', 'Display', 'off', 'MaxIter', 100);
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
        [~, ~, exitflag] = linprog(f, [], [], Aeq, beq, u_lb, u_ub, opt_options);
        if exitflag ~= 1
            inWorkspace = false;
            return
        end
    end
    inWorkspace = true;
    
end