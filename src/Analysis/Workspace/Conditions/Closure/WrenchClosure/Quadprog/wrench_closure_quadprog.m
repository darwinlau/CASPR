function inWorkspace = wrench_closure_quadprog(dynamics,options)
    % Determine necessary variables for test
    L       =   dynamics.L; % Cable Jacobian
    L_rank  =   rank(L');   % Cable Jacobian Rank
    % Test if Jacobian is a positive spanning subspace
    H       =   eye(dynamics.numCables);
    f       =   zeros(dynamics.numCables,1);
    A_eq     =   -dynamics.M\L';
    b_eq     =   zeros(dynamics.numDofs,1);
    lb      =   1e-6*ones(dynamics.numCables,1);
    ub      =   Inf*ones(dynamics.numCables,1);
    [~, exit_type ] = id_qp_matlab(H, f, [], [], A_eq, b_eq, lb, ub, [],options);
    inWorkspace = (exit_type==IDSolverExitType.NO_ERROR) && (L_rank == dynamics.numDofs);
end