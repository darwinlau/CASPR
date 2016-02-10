function [ x_opt, exit_type, comp_time ] = id_fp_1_norm(A_eq, b_eq, xmin, xmax)
    % Find 1-Norm optimal solution
    %--------------------------------------------------------------------------
    % Jihong Zhu
    % 2016/01/21
    % Required function:
    % Cal_ni: calculate the prependicular vector
    % Cal_alpha: calculate alpha 
    %--------------------------------------------------------------------------
    delta = 1e-8;   % Use of delta to aviod numerical error
    m = size(A_eq,1);
    n = size(A_eq,2);
    [Q,R] = qr(A_eq');
    R = R(1:m,1:m);
    M = Q(:,1:m);
    N = Q(:,m+1:n);
    x_p = M/(R')*b_eq;
    l_N = length(N);
    %--------------------------------------------------------------------------
    
    i = 1;
    bi = -1;
    bj = -1;
    for j = 1:l_N
        if N(j,1)/N(j,2)~=N(1,1)/N(1,2)
            N_p = [N(1,:);N(j,:)];
            break;
        end
    end
    % Calculate the intersection use min (max can also do)
    v_ij = N_p\[xmin(i)-x_p(i);xmin(j)-x_p(j)];
    Indexset = zeros(l_N,1);
    for index = 1:l_N
        if ((xmin(index) - x_p(index) - N(index,:) * v_ij <= delta) && (N(index,:) * v_ij - xmax(index) + x_p(index) <= delta))
            Indexset(index) = 1;
            %                 Indexset = [Indexset,index];
        end
    end
    
    v_f = v_ij;
    while 1
        ni_prepend = Cal_ni(N,i,j,bj);
        [alpha,l,bl] = Cal_alpha(N,ni_prepend,i,v_ij,xmin,xmax,x_p);
        v_li = v_ij + alpha * ni_prepend;        % The next point
        if(Indexset(l)==0)
            %                 NotIndexset = complete_index(~ismember(complete_index,Indexset));
            NotIndexset = find(Indexset==0);
            for i1 = 1:length(NotIndexset)
                index = NotIndexset(i1);
                if ((xmin(index) - x_p(index) - N(index,:) * v_li <= delta) && (N(index,:) * v_li - xmax(index) + x_p(index) <= delta))
                    Indexset(index) = 1;
                end
            end
            v_f = v_li;
            v_ij = v_li;
            j = i;
            i = l;
            bj = bi;
            bi = bl;
        else
            if  ~(abs(v_f(1) - v_li(1))<delta && abs(v_f(2) - v_li(2))<delta), % ~strcmp(num2str(v_f),num2str(v_li))==0,
                if(sum(Indexset)==l_N)
                    Cond = KKT_vertex_1_norm(N,l,i,bl,bi);
                    if(Cond == 1)
                        x_opt = x_p + N*v_li;
                        exit_type = IDSolverExitType.NO_ERROR;
                        return;
                    else
                        v_ij = v_li;
                        j = i;
                        i = l;
                        bj = bi;
                        bi = bl;
                    end
                else
                    v_ij = v_li;
                    j = i;
                    i = l;
                    bj = bi;
                    bi = bl;
                end
            else
                if(sum(Indexset)==l_N)
                    Cond = KKT_vertex_1_norm(N,l,i,bl,bi);
                    if Cond ==1,
                        x_opt = x_p + N*v_li;
                        exit_type = IDSolverExitType.NO_ERROR;
                        return;
                    else
                        x_opt = x_p + N*v_li;
                        exit_type = IDSolverExitType.INFEASIBLE;
                        return;
                    end
                else
                    x_opt = x_p + N*v_li;
                    exit_type = IDSolverExitType.INFEASIBLE;
                    return;
                end
            end
        end
    end
end

