% An efficient inverse dynamics solver for CDPR systems.
%
% Please cite the following paper when using this algorithm:
% K. M�ller and C. Reichert and T. Bruckmann. 
% "Analysis of a real-time capable cable force computation method." 
% In Cable-Driven Parallel Robots, pp. 227-238. Springer International 
% Publishing, 2015.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : This is an implementation of the puncture method for that
%                 is to be applied to the solution of the closed form
%                 method.
function [ x_opt, exit_type] = id_cf_ipm(A_eq, b_eq, x_min, x_max)
    % This is the minimum norm solution if there are no constraints
    Ap = A_eq'/(A_eq*A_eq'); x_unconstrained_opt = Ap*b_eq; Ap_active = Ap;
    if((sum(x_unconstrained_opt - x_min < -1e-6)>0)||(sum(x_unconstrained_opt - x_max > 1e-6)>0))
        x_m = 0.5*(x_min + x_max);  
        [n,m] = size(A_eq);
        x_shift = A_eq*x_m;  
        x_temp = x_m + Ap*(b_eq - x_shift);
        index = true(m,1); 
        x_fixed = x_m;
        b_eq_active = b_eq;
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
            A_eq_active = A_eq(:,index);
            Ap_active = A_eq_active'/(A_eq_active*A_eq_active');
            b_eq_active = b_eq_active - x_fixed(i)*A_eq(:,i);
            x_active = x_m(index) + Ap_active*(b_eq_active - A_eq_active*x_m(index));
            x_temp(index) = x_active;
            x_temp(~index) = x_fixed(~index);
        end
        %------------------------------------------------------------------
        % The next two lines do not appear in the original paper. They do
        % appear to provide improved results.
        %------------------------------------------------------------------
        x_unconstrained_opt(index) = Ap_active*b_eq_active;
        x_unconstrained_opt(~index) = x_fixed(~index);
%         x_unconstrained_opt = Ap*b_eq;
        % The rest is consistent with original paper
        x_offset = x_temp - x_unconstrained_opt;
        s = 0;
        for i=1:m
            if(x_unconstrained_opt(i) - x_min(i)<-1e-6)
                s_temp = (x_min(i)-x_unconstrained_opt(i))/x_offset(i);
                if(s_temp > s)
                    s = s_temp;
                end
            end
        end
        % TEST THE SOLUTION VERSUS THE FIXED VALUES
        x_opt = s*x_offset + x_unconstrained_opt;
        exit_type = IDSolverExitType.NO_ERROR;
    else
        x_opt = x_unconstrained_opt;
        exit_type = IDSolverExitType.NO_ERROR;
    end
end