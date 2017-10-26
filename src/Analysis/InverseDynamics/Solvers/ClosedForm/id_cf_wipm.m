% An efficient inverse dynamics solver for CDPR systems.
%
% Please cite the following paper when using this algorithm:
% A. Pott, "An Improved Force Distribution Algorithm for Over-Constrained
% Cable-Driven Parallel Robots", Proceedings of the 6th International
% Workshop on Computational Kinematics (CK2013), pp. 139-146, 2014.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : This is an implementation of the improved closed form
%                 method. This method will produce feasible solutions in a
%                 greater number of poses.
%                 To improve the speed of implementation a rank one update
%                 has been utilised for calculation of the pseduoinverse.
function [ x_opt, exit_type,index_a,x_fixed_a] = id_cf_wipm(A_eq, b_eq, x_min, x_max,index,x_fixed)
    % This is the minimum norm solution if there are no constraints
    Ap = A_eq'/(A_eq*A_eq'); x_unconstrained_opt = Ap*b_eq;
    if((sum(x_unconstrained_opt - x_min < -1e-6)>0)||(sum(x_unconstrained_opt - x_max > 1e-6)>0))
        x_m = 0.5*(x_min + x_max);  
        [n,m] = size(A_eq);
    %     Ap = A_eq'/(A_eq*A_eq');
        A_eq_active = A_eq(:,index);
        Ap_active = A_eq_active'/(A_eq_active*A_eq_active');
        if(sum(index == true) == m)
            b_eq_active = b_eq;
        else
            b_eq_active = b_eq - A_eq(:,~index)*x_fixed(~index);
        end
        x_active = x_m(index) + Ap_active*(b_eq_active - A_eq_active*x_m(index));
        x_temp(index,:) = x_active;
        x_temp(~index) = x_fixed(~index);
        if(isnan(x_temp))
            x_opt = x_temp;
            exit_type = IDSolverExitType.INFEASIBLE;
            return;
        end
        is_not_optimal = 1;
        while(((sum(x_temp - x_min < -1e-6)>0)||(sum(x_temp - x_max > 1e-6)>0))||is_not_optimal)
            if((sum(x_temp - x_min < -1e-6)>0)||(sum(x_temp - x_max > 1e-6)>0))
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
                A_eq_active = A_eq(:,index);
                Ap_active = A_eq_active'/(A_eq_active*A_eq_active');
                b_eq_active = b_eq_active - x_fixed(i)*A_eq(:,i);
                x_active = x_m(index) + Ap_active*(b_eq_active - A_eq_active*x_m(index));
                x_temp(index) = x_active;
                x_temp(~index) = x_fixed(~index);
            else
                % Have a solution just don't know if it is optimal
                W = diag(index);
                mu = -(eye(m) - A_eq'*pinv((A_eq*W)'))*(x_temp-x_m);
                if(sum(mu>1e-6)==0)
                    is_not_optimal = 0;
                else
                    % Find the worst value for mu
                    [~,index_j] = max(mu);
                    % Remove the constraint and update the solution
                    index(index_j) = true;
                    A_eq_active = A_eq(:,index);
                    Ap_active = A_eq_active'/(A_eq_active*A_eq_active');
                    b_eq_active = b_eq_active + x_fixed(index_j)*A_eq(:,index_j);
                    x_fixed(index_j) = 0;                
                    x_active = x_m(index) + Ap_active*(b_eq_active - A_eq_active*x_m(index));
                    x_temp(index) = x_active;
                    x_temp(~index) = x_fixed(~index);
                end
            end
        end
%         x_opt = x_temp;
        index_a = index;
        x_fixed_a = x_fixed;
        %------------------------------------------------------------------
        % The next two lines do not appear in the original paper. They do
        % appear to provide improved results.
        %------------------------------------------------------------------
        x_unconstrained_opt(index) = Ap_active*b_eq_active;
        x_unconstrained_opt(~index) = x_fixed(~index);
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
        % JUST CONFIRM IF x_opt is the same as the actual optimal solution
        exit_type = IDSolverExitType.NO_ERROR;
    else
        x_opt = x_unconstrained_opt;
        exit_type = IDSolverExitType.NO_ERROR;
    end
end