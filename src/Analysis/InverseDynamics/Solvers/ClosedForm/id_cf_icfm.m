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
function [ x_opt, exit_type] = id_cf_icfm(A_eq, b_eq, x_min, x_max)
    x_m = 0.5*(x_min + x_max);  
    [n,m] = size(A_eq);
    x_shift = A_eq*x_m;  
    Ap = A_eq'/(A_eq*A_eq');
    x_temp = x_m + Ap*(b_eq - x_shift);
    if(isnan(x_temp))
        x_opt = x_temp;
        exit_type = IDSolverExitType.INFEASIBLE;
        return;
    end
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
        % Update the solution
        A_eq_active = A_eq(:,index);
        Ap_active = A_eq_active'/(A_eq_active*A_eq_active');
        b_eq_active = b_eq_active - x_fixed(i)*A_eq(:,i);
        x_active = x_m(index) + Ap_active*(b_eq_active - A_eq_active*x_m(index));
        x_temp(index) = x_active;
        x_temp(~index) = x_fixed(~index);
    end
    x_opt = x_temp;
    exit_type = IDSolverExitType.NO_ERROR;
end