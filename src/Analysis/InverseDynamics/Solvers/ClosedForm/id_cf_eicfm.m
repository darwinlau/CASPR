function [ x_opt, exit_type] = id_cf_eicfm(A_eq, b_eq, x_min, x_max)
    x_m = 0.5*(x_min + x_max);  
    [n,m] = size(A_eq);
    x_shift = A_eq*x_m;  
    Ap = A_eq'/(A_eq*A_eq');
    N = eye(m) - Ap*A_eq;
    x_temp = x_m + Ap*(b_eq - x_shift);
    index = true(length(x_min),1); 
    x_fixed = x_m;  
    A_temp = A_eq;
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
        if(sum(index)<n)
            % No redundancy return infeasible solution
            x_opt = x_temp;
            exit_type = IDSolverExitType.INFEASIBLE;
            return;
        end    
        % Update the solution    
        c = -A_temp(:,i); 
        x_shift = x_shift - c*(x_fixed(i) - x_m(i));
        v = Ap*c;
        beta = 1 + v(i); 
        w = N(i,:)';
        G = (1/beta)*w*v'*Ap;
        Ap = Ap + G; A_temp(:,i) = zeros(n,1);
        N = eye(m) - Ap*A_temp;
        x_temp = x_fixed + Ap*(b_eq - x_shift);
    end
    x_opt = x_temp;
    exit_type = IDSolverExitType.NO_ERROR;
end

% Rank one update can be added to improve the system performance
% Intermediate variables for legibility
% N can also be improved computationally