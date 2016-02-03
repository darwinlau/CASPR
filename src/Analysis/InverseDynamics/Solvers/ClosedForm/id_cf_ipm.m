function [ x_opt, exit_type] = id_cf_ipm(A_eq, b_eq, x_min, x_max)
     % This is the minimum norm solution if there are no constraints
    A_pinv = pinv(A_eq); x_unconstrained_opt = A_pinv*b_eq;
    if((sum(x_unconstrained_opt - x_min < -1e-6)>0)||(sum(x_unconstrained_opt - x_max > 1e-6)>0))
        m = length(x_min);
        x_m = 0.5*(x_min + x_max); w_prime = b_eq;
        x_temp = x_m + A_pinv*(w_prime - A_eq*x_m);
        index = true(m,1); x_fixed = zeros(m,1);
        while((sum(x_temp - x_min < -1e-6)>0)||(sum(x_temp - x_max > 1e-6)>0))
            % Find the most violated constraint
            [min_violation,min_i] = max(x_min - x_temp);
            [max_violation,max_i] = max(x_temp - x_max);
            if(max_violation > min_violation)
                i = max_i;
                x_fixed(i) = x_max(i);
            else
                i = min_i;
                x_fixed(i) = x_min(i);
            end
            index(i) = false;
            if(sum(index)<length(b_eq))
                % No redundancy return infeasible solution
                x_opt = x_temp;
                exit_type = IDSolverExitType.INFEASIBLE;
                return;
            end    
            % Update the solution    
            w_prime     =   w_prime - A_eq(:,i)*x_fixed(i);
            x_prime     =   x_m(index) + pinv(A_eq(:,index))*(w_prime - A_eq(:,index)*x_m(index));
            x_temp      =   x_fixed; x_temp(index) = x_prime;
        end
        % These two lines are not in the original paper
        x_free = pinv(A_eq(:,index))*w_prime;
        x_unconstrained_opt = x_fixed; x_unconstrained_opt(index) = x_free;
        % The rest is consistent with original paper
        x_offset = x_temp - x_unconstrained_opt;
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
    else
        x_opt = x_unconstrained_opt;
        exit_type = IDSolverExitType.NO_ERROR;
    end
end

% POSSIBLE CHANGE
% Given that A_pinv is computed, a rank one update can be applied to the
% inverse Jacobian