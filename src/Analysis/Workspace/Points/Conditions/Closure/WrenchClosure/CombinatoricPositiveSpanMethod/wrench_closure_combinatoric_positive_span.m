% An efficient method for evaluating the wrench closure condition using
% nchoosek(m,n) evaluations in the worst case.
%
% Please cite the following paper when using this algorithm:
% W.B. Lim and G. Yang and S.H. Yeo and S.K Mustafa, "A generic 
% force-closure analysis algorithm for cable-driven parallel manipulators",
% Mechanism and Machine Theory 46, pp. 1265-1276, 2011.

%
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : 
function inWorkspace = wrench_closure_combinatoric_positive_span(dynamics)
    % Determine necessary variables for test
    A       =   -dynamics.L_active'; % Cable Jacobian
    n = size(A,1); m = size(A,2); % Size variables
    %% Evaluate Matrix Rank
    if(rank(A)==n)
        %% Determine a_t
        % Find a linearly independent set of vectors
        Ahat = MatrixOperations.FindLinearlyIndependentSet(A);
        a_t             =   -Ahat*ones(n,1); % A pose is WCW if the positive span contains this vector
        combinations    =   nchoosek(1:m,n);
        for i = 1:size(combinations,1)
           G = A(:,combinations(i,:));
%            if(rank(G) == n)
            if(abs(det(G)) >1e-8)
                k        =   G\a_t;
                k_pos    =   k>0;
                k_nonneg =   k>=0;
                if((sum(k_nonneg) == n) && (sum(k_pos) > 0))
                    inWorkspace = true;
                    return;
                end
           end
        end
        inWorkspace = false;
    else
        inWorkspace = false;
    end
end

