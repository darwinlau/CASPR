function [ x_opt, exit_type] = id_cf_icfm(A_eq, b_eq, x_min, x_max)
    x_m = 0.5*(x_min + x_max); w_prime = b_eq;
    x_temp = x_m + pinv(A_eq)*(w_prime - A_eq*x_m);
    index = true(length(x_min),1); x_fixed = zeros(length(x_temp),1);
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
        w_prime     =   w_prime + A_eq(:,i)*x_fixed(i);
        x_prime     =   x_m(index) + pinv(A_eq(:,index))*(w_prime - A_eq(:,index)*x_m(index));
        x_temp      =   x_fixed; x_temp(index) = x_prime;
    end
    x_opt = x_temp;
    exit_type = IDSolverExitType.NO_ERROR;
end

