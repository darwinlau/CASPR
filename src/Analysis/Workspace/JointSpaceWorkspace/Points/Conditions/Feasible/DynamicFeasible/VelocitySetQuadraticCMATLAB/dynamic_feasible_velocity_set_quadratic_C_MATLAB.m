% Dynamic feasible workspace considering a velocity set.
%
% Author         : Chen SONG
% Created        : 2019
% Description    : test the feasibility of a given acceleration set (polytop vertices), an
% external wrench set (polytop vertices) and a velocity set (ellipsoid). MATLAB linear programming
% solver is called to evaluate the feasibility.
function inWorkspace = dynamic_feasible_velocity_set_quadratic_C_MATLAB(Pv_inv, cv, desired_acc_set, desired_we_set, dynamics)

    % gather up the acceleration related wrench points
    M = dynamics.M;
    wr_acc = zeros(size(desired_acc_set));
    for i = 1:size(desired_acc_set, 1)
        wr_acc(i,:) = desired_acc_set(i,:)*M;
    end
    
    % gather up the centrifugal/Coriolis term and gravity related wrench
    wr_G = dynamics.G';
    
    % gather up the velocity related wrench points
    numDofs = dynamics.numDofs;
    N = dynamics.bodyModel.quadraticC();
    C_lb = zeros(dynamics.numDofs, 1)';
    C_ub = zeros(dynamics.numDofs, 1)';
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
    % get the vertices of the C term box
    numVertices = 2^numDofs;
    wr_vel = zeros(numVertices, numDofs);
    offset = double('0');
    for i = 1:numVertices
        beta = double(dec2bin(i-1,numDofs)) - offset;
        wr_vel(i,:) = C_lb.*(~beta) + C_ub.*beta;
%         % an alternative approach but seems to be slower
%         flag_set = dec2bin(i-1,numDofs);
%         flag_vec = str2num(flag_set')';
%         wr_vel(i,:) = C_lb.*(~flag_vec) + C_ub.*flag_vec;
    end
    
    % gather up the external wrench related wrench points
    wr_we = desired_we_set;
    
    % derive the vertices of the required wrench polytope
    wr = minksum(wr_acc, wr_we);
    wr = minksum(wr, wr_vel);
    wr = minksum(wr, wr_G);
    
    vertex_indices = unique(convhulln(wr));
    wr_vertices = wr(vertex_indices, :);
    
    numPoints = size(wr_vertices, 1);
    opt_options = optimoptions('linprog', 'Display', 'off', 'MaxIter', 100);
%     opt_options = cplexoptimset('Display', 'off', 'MaxIter', 100);
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
        [~, ~, exitflag] = linprog(f, [], [], Aeq, beq, u_lb, u_ub, opt_options);
%         [~, ~, exitflag] = cplexlp(f, [], [], Aeq, beq, u_lb, u_ub, opt_options);
        if exitflag ~= 1
            inWorkspace = false;
            return
        end
    end
    inWorkspace = true;
    
end