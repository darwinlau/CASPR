function [alpha,l,bl] = Cal_alpha(N,ni_prepend,i,v_ij,t_min,t_max,t_sol)
% Calculate alpha in order to move to the next intersection
alpha = inf;
index = 1:length(N);
index_new=index(~ismember(index,i));
for ind = 1:length(index_new),
    k = index_new(ind);
    n_k = N(k,:);
    if n_k * ni_prepend == 0
        continue
    elseif n_k * ni_prepend > 0
        if (n_k * v_ij -  t_min(k) + t_sol(k) < -1e-5)
            temp = (t_min(k) - t_sol(k) - n_k * v_ij)/(n_k * ni_prepend);
            bk = -1;
        elseif  t_min(k) - t_sol(k) <= n_k * v_ij && n_k * v_ij < t_max(k) - t_sol(k)
            temp = (t_max(k) - t_sol(k) - n_k * v_ij)/(n_k * ni_prepend);
            bk = 1;            
        else
            continue
        end
    elseif n_k * ni_prepend < 0
        if (n_k * v_ij -   t_max(k) + t_sol(k) > 1e-5),
            temp = (t_max(k) - t_sol(k) - n_k * v_ij)/(n_k * ni_prepend);
            bk = 1;
        elseif  t_min(k) - t_sol(k) < n_k * v_ij && n_k * v_ij <= t_max(k) - t_sol(k)
            temp = (t_min(k) - t_sol(k) - n_k * v_ij)/(n_k * ni_prepend);
            bk = -1;
        else
            continue
        end
    end
    % This above will fail in one situation where the next intersection is
    % on the parallel line
    if temp < alpha
        alpha = temp;
        l = k;
        bl = bk;
    end
    if(alpha<1e-4)
        sfgdlkj
    end
end

end

