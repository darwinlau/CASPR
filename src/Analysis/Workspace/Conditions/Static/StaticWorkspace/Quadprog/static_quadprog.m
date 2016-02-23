function inWorkspace = static_quadprog(dynamics,options)
    % Test if the pose can be an equilibrium pose
%     H       =   eye(dynamics.numCables);
%     f       =   zeros(dynamics.numCables,1);
%     A       =   [];
%     b       =   [];
%     Aeq     =   -dynamics.L';
%     Aeq     =   (abs(Aeq)>1e-9).*Aeq;
%     % Check each column for nan
%     for i = 1:size(Aeq,2)
%         if(isnan(Aeq(1,i)))
%             Aeq(:,i) = zeros(size(Aeq,1),1);
%         end
%     end
%     beq     =   dynamics.G;
%     lb      =   dynamics.cableDynamics.forcesMin;
%     ub      =   dynamics.cableDynamics.forcesMax;
%     % FILTER FOR NANs.  This should be improved at a later time
%     [~,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
%     if(exit_flag == 1)
%         inWorkspace = 1;
%     else
%         inWorkspace = 0;
%     end
    weights = ones(dynamics.numCables,1);
    o = IDObjectiveMinQuadCableForce(weights);
    q = IDSolverQuadProg(dynamics,o, ID_QP_SolverType.MATLAB);
%     exit_flag
    [~,~,id_exit_type,~] = q.resolve(dynamics.q,dynamics.q_dot,dynamics.q_ddot,dynamics.W_e);
    if(id_exit_type == IDSolverExitType.NO_ERROR)
        inWorkspace = true;
    else
        inWorkspace = false;
    end
end