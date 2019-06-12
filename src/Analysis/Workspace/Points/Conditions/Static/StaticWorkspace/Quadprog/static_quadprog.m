% quadratic program implementation for evaluating static workspace.
% Author         : Jonathan EDEN
% Created        : 2015
% Description    : Implementation of static workspace analysis using a QP
function inWorkspace = static_quadprog(dynamics)
    % Test if the pose can be an equilibrium pose
    weights = ones(dynamics.numCables,1);
    o = IDObjectiveMinQuadCableForce(weights);
    q = IDSolverQuadProg(dynamics,o, ID_QP_SolverType.MATLAB);
    [~,~,~,id_exit_type,~] = q.resolve(dynamics.q,dynamics.q_dot,dynamics.q_ddot,dynamics.W_e);
    if(id_exit_type == IDSolverExitType.NO_ERROR)
        inWorkspace = true;
    else
        inWorkspace = false;
    end
end