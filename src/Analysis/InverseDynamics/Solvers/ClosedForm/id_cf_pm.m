function [ x_opt, exit_type ] = id_cf_pm(A_eq, b_eq, x_min, x_max)
    % This is the minimum norm solution if there are no constraints
    A_pinv = pinv(A_eq); x_unconstrained_opt = A_pinv*b_eq;
    if((sum(x_unconstrained_opt - x_min < -1e-6)>0)||(sum(x_unconstrained_opt - x_max > 1e-6)>0))
        % Unconstrained minimum is not feasible
        m = length(x_min);
        x_m = 0.5*(x_min + x_max); A_pinv = pinv(A_eq);
        x_offset = (eye(m)-A_pinv*A_eq)*x_m; 
        x_temp = x_offset + x_unconstrained_opt;
        if((sum(x_temp - x_min < -1e-6)>0)||(sum(x_temp - x_max > 1e-6)>0))
            % Solution is not feasible
            x_opt = x_temp;
            exit_type = IDSolverExitType.INFEASIBLE;
        else
            % Determine the point of intersection
            s = 0;
            for i=1:m
                if(x_unconstrained_opt(i) - x_min(i)<0)
                    s_temp = (x_min(i)-x_unconstrained_opt(i))/x_offset(i);
                    if(s_temp > s)
                        s = s_temp;
                    end
                end
            end
            x_opt = s*x_offset + x_unconstrained_opt;
            exit_type = IDSolverExitType.NO_ERROR;
        end
    else
        x_opt = x_unconstrained_opt;
        exit_type = IDSolverExitType.NO_ERROR;
    end
end

