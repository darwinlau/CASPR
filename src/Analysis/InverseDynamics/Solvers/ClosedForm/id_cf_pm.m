% An efficient inverse dynamics solver for CDPR systems.
%
% Please cite the following paper when using this algorithm:
% K. Müller and C. Reichert and T. Bruckmann. 
% "Analysis of a real-time capable cable force computation method." 
% In Cable-Driven Parallel Robots, pp. 227-238. Springer International 
% Publishing, 2015.
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : This is an implementation of the puncture method for,
%                 is to be applied to the solution of the closed form
%                 method.
function [ x_opt, exit_type ] = id_cf_pm(A_eq, b_eq, x_min, x_max)
    % This is the minimum norm solution if there are no constraints
    x_unconstrained_opt = pinv(A_eq)*b_eq; m = size(A_eq,2);
    if((sum(x_unconstrained_opt - x_min < -1e-6)>0)||(sum(x_unconstrained_opt - x_max > 1e-6)>0))
        % Unconstrained minimum is not feasible
        [x_temp, exit_type] = id_cf_cfm(A_eq, b_eq, x_min, x_max);
        if(exit_type == IDSolverExitType.INFEASIBLE)
            x_opt = x_temp;
        else
            % Determine the point of intersection
            s = 0;
            x_offset = x_temp - x_unconstrained_opt;
            for i=1:m
                if(x_unconstrained_opt(i) - x_min(i)<0)
                    s_temp = (x_min(i)-x_unconstrained_opt(i))/x_offset(i);
                    if(s_temp > s)
                        s = s_temp;
                    end
                end
            end
            x_opt = s*x_offset + x_unconstrained_opt;
        end
    else
        x_opt = x_unconstrained_opt;
        exit_type = IDSolverExitType.NO_ERROR;
    end
end

