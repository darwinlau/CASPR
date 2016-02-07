% An efficient method for evaluating the wrench closure condition using
% nchoosek(m,n+1) evaluations in the worst case.
%
% Please cite the following paper when using this algorithm:
% B. Ouyang and W.W. Shang, "A new computation method for the force-closure workspace of 
% cable-driven parallel manipulators", Robotica 33.3, pp. 537-547, 2015.

%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : 
function inWorkspace = wrench_closure_shang(dynamics,options)
    % Determine necessary variables for test
    A       =   -dynamics.L'; % Cable Jacobian
%     A
    [n,m] = size(A); % Size variables
    %% Evaluate Matrix Rank
    if(rank(A)==n)
        %% Determine a_t
        % Find a linearly independent set of vectors
        Ahat = MatrixOperations.FindLinearlyIndependentSet(A);
        a_t = -Ahat*ones(n,1); % A pose is WCW if the positive span contains this vector
        %% Detect the dot product sign
        % Take the dot product of each column with a_t
        DP = A'*a_t;
        % Check the sign of DP
        if((sum(DP<=0)==m) || (sum(DP>=0)==m))
            inWorkspace = 0;
        else
            % Determine all of the possible combinations
            index = DP>0;
            A_pos = A(:,index);
            A_neg = A(:,~index);
            l_pos = size(A_pos,2);
            l_neg = m - l_pos;
            is_pos_smaller = 0;
            if((l_pos>n-1)&&(l_neg>n-1))
                p = n;
            else
                p = min([l_neg,l_pos]);
                if(p==l_pos)
                    is_pos_smaller = 1;
                end
            end
            % For each combination evaluate the positivity condition
            for i=1:p
                a_l = is_pos_smaller*i + ~is_pos_smaller*(n+1-i);
                possible_pos = nchoosek(1:l_pos,a_l);
                for j=1:size(possible_pos,1)
                    % Map a combination into a set of vectors
                    pos_set = A_pos(:,possible_pos(j,:));
                    possible_neg = nchoosek(1:l_neg,n+1-a_l);
                    positivity_condition_failed = 0;
                    for k =1:size(possible_neg,1)
                        neg_set = A_neg(:,possible_neg(k,:));
                        A_i = [pos_set,neg_set];
                        A_p = A_i'/(A_i*A_i');
                        N_i = null(A_i);
                        lambda = A_p*a_t; ratio_pos = -Inf;
                        ratio_neg = Inf; 
                        for l = 1:n+1
                            if(N_i(l) > 1e-6)
                                temp = -lambda(l)/N_i(l);
                                if(temp > ratio_pos)
                                    ratio_pos = temp;
                                end
                            elseif(N_i(l) < -1e-6)
                                temp = -lambda(l)/N_i(l);
                                if(temp < ratio_neg)
                                    ratio_neg = temp;
                                end
                            else
                                if(lambda(l) < 0)
                                    positivity_condition_failed = 1;
                                    break;
                                end
                            end
                        end
                        if(positivity_condition_failed)
                            break;
                        end
                        if(ratio_pos <= ratio_neg)
                            inWorkspace = 1;
                            return;
                        end
                    end
                end
            end
            inWorkspace = 0;
        end
    else
        inWorkspace = 0;
    end
end

