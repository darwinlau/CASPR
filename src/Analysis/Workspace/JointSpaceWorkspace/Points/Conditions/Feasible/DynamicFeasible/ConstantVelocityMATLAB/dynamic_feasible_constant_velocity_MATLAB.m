% Dynamic feasible workspace considering a constant velocity.
%
% Please cite the following paper when using this algorithm:
% L. Gagliardini, M. Gouttefarde, S. Caro "Determination of a Dynamic 
% Feasible Workspace for Cable-Driven Parallel Robots", 
% in Advances in Robot Kinematics 2016 (pp. 361-370). Springer, Cham
%
% Author         : Chen SONG
% Created        : 2019
% Description    : test the feasibility of a given acceleration box, an
% external wrench box and a constant velocity. MATLAB linear programming
% solver is called to evaluate the feasibility.
function inWorkspace = dynamic_feasible_constant_velocity_MATLAB(desired_vel_set_vertices, desired_acc_set_vertices, desired_we_set_vertices, dynamics)

    % gather up the acceleration related wrench points
    M = dynamics.M;
    wr_acc = zeros(size(desired_acc_set_vertices));
    for i = 1:size(desired_acc_set_vertices, 1)
        wr_acc(i,:) = desired_acc_set_vertices(i,:)*M;
    end
    
    % gather up the centrifugal/Coriolis term and gravity related wrench
    N = dynamics.bodyModel.quadraticC();
    wr_CG = dynamics.G';
    for i = 1:dynamics.numDofs
        wr_CG(i) = wr_CG(i) + desired_vel_set_vertices*N(:,:,i)*desired_vel_set_vertices';
    end
    
    % gather up the external wrench related wrench points
    wr_we = desired_we_set_vertices;
    
    % derive the vertices of the required wrench polytope
    wr = minksum(wr_acc, wr_we);
    wr = minksum(wr, wr_CG);
    vertex_indices = unique(convhulln(wr));
    wr_vertices = wr(vertex_indices, :);
    
    numPoints = size(wr_vertices, 1);
    opt_options = optimoptions('linprog', 'Display', 'off', 'MaxIter', 100);
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
        if exitflag ~= 1
            inWorkspace = false;
            return
        end
    end
    inWorkspace = true;
    
end