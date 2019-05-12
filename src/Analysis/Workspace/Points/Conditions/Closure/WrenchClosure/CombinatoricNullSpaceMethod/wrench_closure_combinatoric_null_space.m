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
function inWorkspace = wrench_closure_combinatoric_null_space(dynamics)
    % Determine necessary variables for test
    A       =   -dynamics.L_active'; % Cable Jacobian
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
        index = DP<=0; l_neg = sum(index);
        if(l_neg==m)
            inWorkspace = false;
        else
            % Determine all of the possible combinations
            A_pos = A(:,~index);
            A_neg = A(:,index);
            l_pos = m - l_neg;
            if(l_pos>n-1)
                p_up = n;
            else
                p_up = l_pos;
            end
            if(l_neg >= n)
                p_down = 1;
            else
                p_down = n + 1 - l_neg;
            end
            % For each combination evaluate the positivity condition
            for i=p_up:-1:p_down
                possible_pos = nchoosek(1:l_pos,i);
                for j=1:size(possible_pos,1)
                    % Map a combination into a set of vectors
                    pos_set = A_pos(:,possible_pos(j,:));
                    possible_neg = nchoosek(1:l_neg,n+1-i);
                    positivity_condition_failed = 0;
                    for k =1:size(possible_neg,1)
                        neg_set = A_neg(:,possible_neg(k,:));
                        A_i = [pos_set,neg_set];
                        [Q,R] = qr(A_i');
                        A_p = Q(:,1:n)/(R(1:n,1:n)');
                        N_i = Q(:,n+1);
%                         A_p = A_i'/(A_i*A_i');
%                         N_i = null(A_i);
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
                            positivity_condition_failed = 0;
                        elseif(ratio_pos <= ratio_neg)
                            inWorkspace = true;
                            return;
                        end
                    end
                end
            end
            inWorkspace = false;
        end
    else
        inWorkspace = false;
    end
end

