% The MATLAB implementation of the optimally safe solvers
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : MATLAB implementation of optimally safe solvers
function [ x_opt, exit_type] = id_os_matlab(A_eq, b_eq, xmin, xmax,alpha,options)
    % Find the optimally safe cable tension by solving a linear progrm
    m = length(xmin);
    f = [zeros(m,1);-1];
    A_eq = [A_eq,zeros(size(A_eq,1),1)];
    A_ineq = [-eye(m),ones(m,1);eye(m),alpha*ones(m,1)];
    b_ineq = [-xmin;xmax];
    [x,~,exit_flag] = linprog(f,A_ineq,b_ineq,A_eq,b_eq,[],[],options);
    if((x(m+1)<0)||(exit_flag~=1))
        % The problem is infeasible
        x_opt = x(1:m);
        exit_type = IDSolverExitType.INFEASIBLE;
    else
        x_opt = x(1:m);
        exit_type = IDSolverExitType.NO_ERROR;
    end
end