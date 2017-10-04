% The MATLAB implementation of the optimally safe solvers with warm start
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : MATLAB implementation of optimally safe solvers with warm
% starting
function [ x_opt, exit_type, x_prev_new, active_set_new] = id_os_efficient(A_eq, b_eq, xmin, xmax,alpha,x_prev,active_set,options)
    % Find the optimally safe cable tension by solving a linear progrm
    m = length(xmin); n = size(A_eq,1); r = m-n;
    f = [zeros(r,1);-1];
    CASPR_log.Assert(rank(A_eq)==n,'Algorithm does not work for singular matrices')
    [B,H] = MatrixOperations.FindLinearlyIndependentSet(A_eq);
    BinvH = B\H;
    A_ineq = [-eye(r),ones(r,1);BinvH,ones(n,1);eye(r),alpha*ones(r,1);-BinvH,alpha*ones(n,1)];
    b_ineq = [-xmin(1)*ones(r,1);-xmin(1)*ones(n,1) + B\b_eq;xmax(1)*ones(r,1);xmax(1)*ones(n,1) - B\b_eq];
    if(isempty(active_set))
        [x,~,exit_flag,~,lambda] = linprog(f,A_ineq,b_ineq,[],[],[],[],[],options);
        active_set_new = lambda.ineqlin>1e-6;
    else
        [m1,n1] = size(A_ineq(active_set,:));
        if(m1>=n1)
            z = -pinv(A_ineq(active_set,:)')*f;
            x0 = pinv(A_ineq(active_set,:))*b_ineq(active_set);
            if((sum(z<-1e-6)==0)&&(sum(A_ineq*x0-b_ineq>1e-6)==0))
                % Optimal feasible solution already
                x = x0; % This is the solution to x for given active set. Modify to give the active set and test
                active_set_new = active_set;
                exit_flag = 1;
            else
                % First check if the solution is feasible
                if(sum(A_ineq*x0>b_ineq)==0)
                    % The solution is feasible so solve the LP
                    [x,~,exit_flag,~,lambda] = linprog(f,A_ineq,b_ineq,[],[],[],[],x0,options);
                    active_set_new = lambda.ineqlin>1e-6;
                else
                    % The solution is not feasible so determine a feasible
                    % solution
                   S_max = min((b_ineq - A_ineq(:,1:r)*x_prev(1:r))./A_ineq(:,r+1));
                   [x,~,exit_flag,~,lambda] = linprog(f,A_ineq,b_ineq,[],[],[],[],[x_prev(1:r);S_max],options);
                   active_set_new = lambda.ineqlin>1e-6;
                end
            end
        else
            [x,~,exit_flag,~,lambda] = linprog(f,A_ineq,b_ineq,[],[],[],[],[],options);
            active_set_new = lambda.ineqlin>1e-6;
        end
    end
    if((x(r+1)<0)||(exit_flag~=1))
        % The problem is infeasible
        x_opt = [B\(b_eq-H*x(1:r));x(1:r)];
        x_prev_new = x;
        exit_type = IDSolverExitType.INFEASIBLE;
        active_set_new = [];
    else
        x_opt = [B\(b_eq-H*x(1:r));x(1:r)];
        x_prev_new = x;
        exit_type = IDSolverExitType.NO_ERROR;
    end
end
