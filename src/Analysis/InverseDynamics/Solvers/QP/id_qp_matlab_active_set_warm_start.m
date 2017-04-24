% The MATLAB implementation of quadratic programming solvers with warm
% starting
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : MATLAB implementation of QP solvers with warm starting.
function [ x_opt, exit_type,active_set_new] = id_qp_matlab_active_set_warm_start(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0,active_set,options)
    CASPR_log.Assert(det(A)>=1e-6,'Efficient Method does not work for singular A');
    CASPR_log.Assert(isempty(A_ineq)&&isempty(b_ineq),'Efficient method cannot handle inequality constraints');
%     [n,m] = size(A_eq);
%     C = [eye(m);-eye(m)]; d = [xmin;-xmax];
%     [x_opt,exitflag,active_set_new] = mpcqpsolver(A,b,C,d,A_eq,b_eq,active_set,options);
    if(isempty(active_set))
        [x_opt, ~, exitflag, ~, lambda] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
        if(exitflag<=0)
            active_set_new = [];
        else 
            active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
        end
%         optisolver = opti('qp', A, b, 'ineq', A_ineq, b_ineq, 'eq', A_eq, b_eq, 'bounds', xmin, xmax, 'options', options);
%         [x_opt, ~, exitflag,info] = solve(optisolver, x0);
%         lambda = info.Lambda;
%         active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
    elseif((sum(b ~=0)==0)&&isdiag(A))
        % Use the active set to get the optimal solution
        m = length(xmin); Ainv = eye(m)/A;
        joint_saturated =   active_set(1:m) |  active_set(m+1:2*m);
        q_n             =   active_set(1:m).*xmin + active_set(m+1:2*m).*xmax;
        W       =   diag(~joint_saturated);
        AW = A_eq*W;
        AWinv = Ainv*AW'/(AW*Ainv*AW');
        Pbar    =   eye(m) - AWinv*A_eq;
        x0  =   AWinv*b_eq + Pbar*q_n;
        mu = -Pbar'*A*x0;
        if(sum(x0-xmin<-1e-6)||sum(xmax-x0<-1e-6)||sum(-mu.*(q_n==xmin) + mu.*(q_n==xmax)<-1e-6))
            if(sum(x0-xmin<-1e-6)||sum(xmax - x0<-1e-6))
                % For the moment call qp
                [x_opt, ~, exitflag,~,lambda] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
                active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
%                 optisolver = opti('qp', A, b, 'ineq', A_ineq, b_ineq, 'eq', A_eq, b_eq, 'bounds', xmin, xmax, 'options', options);
%                 [x_opt, ~, exitflag,info] = solve(optisolver, x0);
%                 lambda = info.Lambda;
%                 active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
            else
                % The issue is with optimality
                joint_saturated = (~diag(W)).*(-mu.*(q_n==xmin) + mu.*(q_n==xmax)>-1e-6);
                q_n = joint_saturated.*q_n;
                W       =   diag(~joint_saturated);
                AW = A_eq*W;
                AWinv = Ainv*(AW)'/(AW*Ainv*AW');
                Pbar    =   eye(m) - AWinv*A_eq;
                x_opt  =   AWinv*b_eq + Pbar*q_n;
                active_set_new = [joint_saturated.*(q_n==xmin);joint_saturated.*(q_n==xmax)];
                exitflag = 1;
%                 [x_opt_quad, ~, ~,~,lambda] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
%                 W = diag(diag(W).*)
            end
        else
            x_opt = x0;
            active_set_new = active_set;
            exitflag = 1;
        end
    else
        % Solve for lambda and x
        % THIS COMPONENT NEEDS TO BE BETTER TESTED WHEN A WIDER RANGE OF
        % OBJECTIVES HAVE BEEN ADDED
        [n,m] = size(A_eq);
        joint_saturated =   active_set(1:m) |  active_set(m+1:2*m);
        q_n             =   active_set(1:m).*xmin + active_set(m+1:2*m).*xmax;
        W = ~joint_saturated;
        AW = A_eq(:,W);
        HW = A(W,W);
        HWinv = eye(m-sum(joint_saturated))/HW;
        AWHWAWinv = eye(n)/(AW*HWinv*AW');
        x_sub = -(HWinv - HWinv*AW'*AWHWAWinv*AW*HWinv)*(b(W) + A(W,~W)*q_n(~W)) + HWinv*AW'*AWHWAWinv*(b_eq - A_eq(:,~W)*q_n(~W));
        lambda = -AWHWAWinv*AW*HWinv*(b(W) + A(W,~W)*q_n(~W)) - AWHWAWinv*(b_eq - A_eq(:,~W)*q_n(~W));
        x0 = q_n; x0(W) = x_sub;
        mu = -A*x0 - A_eq'*lambda;
        if(sum(x0-xmin<-1e-6)||sum(xmax-x0<-1e-6)||sum(-mu.*(q_n==xmin) + mu.*(q_n==xmax)<-1e-6))
            if(sum(x0-xmin<-1e-6)||sum(xmax - x0<-1e-6))
                % For the moment call qp
                [x_opt, ~, exitflag,~,lambda] = quadprog(A, b, A_ineq, b_ineq, A_eq, b_eq, xmin, xmax, x0, options);
                active_set_new = [lambda.lower > 1e-6; lambda.upper > 1e-6];
            else
                % The issue is with optimality
                joint_saturated = (~W).*(-mu.*(q_n==xmin) + mu.*(q_n==xmax)>-1e-6);
                W = ~joint_saturated;
                AW = A_eq(:,W);
                HW = A(W,W);
                HWinv = eye(size(HW))/HW;
                AWHWAWinv = eye(n)/(AW*HWinv*AW');
                x_sub = -(HWinv - HWinv*AW'*AWHWAWinv*AW*HWinv)*(b(W) + A(W,~W)*q_n(~W)) + HWinv*AW'*AWHWAWinv*(b_eq - A_eq(:,~W)*q_n(~W));
                x_opt = q_n; x_opt(W) = x_sub;
                exitflag = 1;
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
            CASPR_log.Info('Max iteration limit reached');
            exit_type = IDSolverExitType.ITERATION_LIMIT_REACHED;
        case -2
            CASPR_log.Info('Problem infeasible');
            exit_type = IDSolverExitType.INFEASIBLE;
            x_opt = xmin;
        otherwise
            CASPR_log.Info(sprintf('Other error : Code %d', exitflag));
            exit_type = IDSolverExitType.SOLVER_SPECIFIC_ERROR;
    end
end
