function inWorkspace = static_quadprog(dynamics,options)
    % Test if the pose can be an equilibrium pose
    H       =   eye(dynamics.numCables);
    f       =   zeros(dynamics.numCables,1);
    A       =   [];
    b       =   [];
    Aeq     =   -dynamics.L';
    Aeq     =   (abs(Aeq)>1e-9).*Aeq;
    beq     =   dynamics.G;
    lb      =   dynamics.cableDynamics.forcesMin;
    ub      =   dynamics.cableDynamics.forcesMax;
    [~,~,exit_flag] = quadprog(H,f,A,b,Aeq,beq,lb,ub,[],options);
    if(exit_flag == 1)
        inWorkspace = 1;
    else
        inWorkspace = 0;
    end
end