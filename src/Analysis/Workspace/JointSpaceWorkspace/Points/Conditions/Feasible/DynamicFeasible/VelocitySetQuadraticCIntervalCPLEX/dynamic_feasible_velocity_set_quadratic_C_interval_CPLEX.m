% Dynamic feasible workspace considering a velocity set.
%
% Author         : Chen SONG
% Created        : 2019
% Description    : test the feasibility of a given acceleration set 
% (interval), an external wrench set (interval) and a velocity set 
% (ellipsoid). The required wrench set is relaxed into an interval for 
% computing speed. CPLEX linear programming solver is called to evaluate the feasibility.
function inWorkspace = dynamic_feasible_velocity_set_quadratic_C_interval_CPLEX(Pv_inv, cv, desired_acc_set, desired_we_set, dynamics)

    % gather up the acceleration related wrench interval
    wr_acc_intval = dynamics.M*desired_acc_set;
    
    % gather up the centrifugal/Coriolis term and gravity related wrench
    wr_G_intval = intval(dynamics.G);
    
    % gather up the velocity related wrench points
    N = dynamics.bodyModel.quadraticC();
    C_lb = zeros(dynamics.numDofs, 1);
    C_ub = zeros(dynamics.numDofs, 1);
    % get the over-estimated C term box
    for i = 1:dynamics.numDofs
        Ni = 0.5*(N(:,:,i) + N(:,:,i)');
        tmpVecNorm = norm(2*cv'*Ni*Pv_inv);
        tmpMat = Pv_inv'*Ni*Pv_inv;
        tmpConst = cv'*Ni*cv;
        lambda = eig(tmpMat);
        C_lb(i) = min([0, min(lambda)-tmpVecNorm + tmpConst]);
        C_ub(i) = max([0, max(lambda)+tmpVecNorm + tmpConst]);
    end
    wr_vel_intval = infsup(C_lb, C_ub);
    
    % gather up the external wrench related wrench points
    wr_we_intval = desired_we_set;
    
    
    wr_intval = wr_acc_intval + wr_G_intval + wr_vel_intval + wr_we_intval;
    
    % get the vertices of the C term box
    numDofs = dynamics.numDofs;
    numVertices = 2^numDofs;
    wr_vertices = zeros(numVertices, numDofs);
    lb = wr_intval.inf';
    ub = wr_intval.sup';
    offset = double('0');
    for i = 1:numVertices
        beta = double(dec2bin(i-1,numDofs)) - offset;
        wr_vertices(i,:) = lb.*(~beta) + ub.*beta;
%         % an alternative approach but seems to be slower
%         flag_set = dec2bin(i-1,numDofs);
%         flag_vec = str2num(flag_set')';
%         wr_vertices(i,:) = lb.*(~flag_vec) + ub.*flag_vec;
    end
    
    numPoints = size(wr_vertices, 1);
%     opt_options = optimoptions('linprog', 'Display', 'off', 'MaxIter', 100);
    opt_options = cplexoptimset('Display', 'off', 'MaxIter', 100);
    f = zeros(dynamics.numActuatorsActive,1);
    u_lb = dynamics.actuationForcesMin;
    u_ub = dynamics.actuationForcesMax;
%     safety_factor = 1e-3;
%     half_gap = (u_ub - u_lb)/2*(1-safety_factor);
%     u_lb = (u_ub + u_lb)/2-half_gap;
%     u_ub = (u_ub + u_lb)/2+half_gap;
    Aeq = [-dynamics.L', dynamics.A];
    for i = 1:numPoints
        beq = wr_vertices(i,:)';
%         [~, ~, exitflag] = linprog(f, [], [], Aeq, beq, u_lb, u_ub, opt_options);
        [~, ~, exitflag] = cplexlp(f, [], [], Aeq, beq, u_lb, u_ub, opt_options);
        if exitflag ~= 1
            inWorkspace = false;
            return
        end
    end
    inWorkspace = true;
    
end