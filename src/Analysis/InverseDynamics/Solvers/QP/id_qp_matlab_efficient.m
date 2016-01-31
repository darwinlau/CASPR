function [ x_opt, exit_type,active_set_new] = id_qp_matlab_efficient(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0,active_set,options)
%     options = optimoptions('quadprog', 'Display', 'off', 'MaxIter', 100);
    if(isempty(active_set))
        [x_opt, ~, exitflag,~,lambda] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
        active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
    else
        % Use the active set to get the optimal solution
        m = length(xmin);
        joint_saturated =   active_set(1:m) |  active_set(m+1:2*m);
        q_n             =   active_set(1:m).*xmin + active_set(m+1:2*m).*xmax;
        W       =   diag(~joint_saturated);
        AW = A_eq*W;
        AWinv = pinv(AW);
        Pbar    =   eye(m) - AWinv*A_eq;
        x0  =   AWinv*b_eq + Pbar*q_n;
        mu = -Pbar'*x0;
        if(sum(x0-xmin<-1e-6)||sum(xmax-x0<-1e-6)||sum(-mu.*(q_n==xmin) + mu.*(q_n==xmax)<-1e-6))
            if(sum(x0-xmin<-1e-6)||sum(xmax - x0<-1e-6))
                % For the moment call qp
                [x_opt, ~, exitflag,~,lambda] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
                active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
            else
                % The issue is with optimality
%                 [x_opt, ~, exitflag,~,lambda] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
%                 active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
                joint_saturated = (~diag(W)).*(-mu.*(q_n==xmin) + mu.*(q_n==xmax)>-1e-6);
                W       =   diag(~joint_saturated);
                AW = A_eq*W;
                AWinv = pinv(AW);
                Pbar    =   eye(m) - AWinv*A_eq;
                x_opt  =   AWinv*b_eq + Pbar*q_n;
                active_set_new = [joint_saturated.*(q_n==xmin);joint_saturated.*(q_n==xmax)];
                exitflag = 1;
%                 W = diag(diag(W).*)
            end
        else
            x_opt = x0;
            active_set_new = active_set;
            exitflag = 1;
        end
    end
        
    switch exitflag
        case 1
            exit_type = IDSolverExitType.NO_ERROR;
        case 0
            fprintf('Max iteration limit reached\n');
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -2
            fprintf('Problem infeasible\n');
            exit_type = IDSolverExitType.INFEASIBLE;
            x_opt = xmin;
        otherwise
            fprintf('Other error : Code %d\n', exit_flag);
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
    end
end

