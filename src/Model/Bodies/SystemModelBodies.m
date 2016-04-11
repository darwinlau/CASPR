% System kinematics of the bodies for the system
%
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102?1113,
% Oct. 2013.
%
% Author        : Darwin LAU
% Created       : 2011
% Description    :
%    Data structure that represents the kinematics of the bodies of the
% system, encapsulated within an array of BodyKinematics object. Also
% provides global matrices for the entire rigid body kinematics system.
classdef SystemModelBodies < handle
    properties
        q_ddot                  % Acceleration of joint space coordiantes
    end
    
    properties (SetAccess = private)
        % Objects
        bodies                  % Cell array of BodyModel objects

        % Generalised coordinates of the system
        q                       % Joint space coordinates
        q_dot                   % Derivatives of joint space coordinates
        
        % Graphs
        connectivityGraph       % p x p connectivity matrix, if (i,j) = 1 means link i-1 is the parent of link j
        bodiesPathGraph         % p x p matrix that governs how to track to particular bodies, (i,j) = 1 means that to get to link j we must pass through link i

        % Operational Space coordinates of the system
        y       = [];           % Operational space coordinates
        y_dot   = [];           % Operational space velocity
        y_ddot  = [];           % Operational space acceleration
        
        % Jacobian Matrices - These matrices should probably be computed as
        % needed (dependent variable), but if it is a commonly used matrix 
        % (i.e. accessed multiple times even if the system state does not 
        % change) then storing it would be more efficient. However this 
        % means that update must be performed through this class' update 
        % function and not on the body's update directly. This makes sense 
        % since just updating one body without updating the others would 
        % cause inconsistency anyway.
        S                       % S matrix representing relationship between relative body velocities (joint) and generalised coordinates velocities
        S_dot                   % Derivative of S
        P                       % 6p x 6p matrix representing mapping between absolute body velocities (CoG) and relative body velocities (joint)
        W                       % W = P*S : 6p x n matrix representing mapping \dot{\mathbf{x}} = W \dot{\mathbf{q}}
        J     = [];             % J matrix representing relationship between generalised coordinate velocities and operational space coordinates
        J_dot = [];             % Derivative of J
        T     = [];             % Projection of operational coordinates

        % Absolute CoM velocities and accelerations (linear and angular)
        x_dot                   % Absolute velocities
        x_ddot                  % Absolute accelerations (x_ddot = W(q)*q_ddot + C_a(q,q_dot))
        C_a                     % Relationship between body and joint accelerations \ddot{\mathbf{x}} = W \ddot{\mathbf{q}} + C_a

        % Degrees of freedom
        numDofs
        numDofVars
        numOPDofs
        numLinks
        
        % Mass matrix
        massInertiaMatrix = [];       % Mass-inertia 6p x 6p matrix
                
        % M_y y_ddot + C_y + G_y = W (operational space)
        M_y = [];
        C_y = [];
        G_y = [];
        
        % M_b * q_ddot + C_b = G_b + w_b - V^T f (forces in body space)
        M_b = [];                        % Body mass-inertia matrix
        C_b = [];                        % Body C matrix
        G_b = [];                        % Body G matrix
        
        % M*q_ddot + C + G + w_e = - L^T f (forces in joint space)
        M = [];
        C = [];
        G = [];
        W_e = [];
    end

    properties (Dependent)
        q_default
        q_dot_default
        q_ddot_default
        q_lb
        q_ub
        % Generalised coordinates time derivative (for special cases q_dot does not equal q_deriv)
        q_deriv
    end
    
    properties 
        occupied                     % An object to keep flags for whether or not matrices are occupied
    end

    methods
        function b = SystemModelBodies(bodies)
            num_dofs = 0;
            num_dof_vars = 0;
            num_op_dofs = 0;
            for k = 1:length(bodies)
                num_dofs = num_dofs + bodies{k}.numDofs;
                num_dof_vars = num_dof_vars + bodies{k}.numDofVars;
            end
            b.bodies = bodies;
            b.numDofs = num_dofs;
            b.numDofVars = num_dof_vars;
            b.numOPDofs = num_op_dofs;
            b.numLinks = length(b.bodies);

            b.connectivityGraph = zeros(b.numLinks, b.numLinks);
            b.bodiesPathGraph = zeros(b.numLinks, b.numLinks);
            b.S = zeros(6*b.numLinks, b.numDofs);
            b.P = zeros(6*b.numLinks, 6*b.numLinks);
            b.W = zeros(6*b.numLinks, b.numDofs);
            b.T = MatrixOperations.Initialise([0,6*b.numLinks],0);

            % Connects the objects of the system and create the
            % connectivity and body path graphs
            b.formConnectiveMap();
            
            b.occupied = BodyFlags();
        end

        % Update the kinematics of the body kinematics for the entire
        % system using the generalised coordinates, velocity and
        % acceleration. This update function should also be called to
        % update the entire system, rather than calling the update function
        % for each body directly.
        function update(obj, q, q_dot, q_ddot, w_ext)
            % Assign q, q_dot, q_ddot
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            obj.W_e = w_ext;
            is_symbolic = isa(q, 'sym');

            % Update each body first
            index_vars = 1;
            index_dofs = 1;
            for k = 1:obj.numLinks
                q_k = q(index_vars:index_vars+obj.bodies{k}.joint.numVars-1);
                q_dot_k = q_dot(index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1);
                q_ddot_k = q_ddot(index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1);
                obj.bodies{k}.update(q_k, q_dot_k, q_ddot_k);
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
                index_dofs = index_dofs + obj.bodies{k}.joint.numDofs;
            end

            % Now the global system updates
            % Set bodies kinematics (rotation matrices)
            for k = 1:obj.numLinks
                parent_link_num = obj.bodies{k}.parentLinkId;
                assert(parent_link_num < k, 'Problem with numbering of links with parent and child');

                % Determine rotation matrix
                % Determine joint location
                if parent_link_num > 0
                    obj.bodies{k}.R_0k = obj.bodies{parent_link_num}.R_0k*obj.bodies{k}.joint.R_pe;
                    obj.bodies{k}.r_OP = obj.bodies{k}.joint.R_pe.'*(obj.bodies{parent_link_num}.r_OP + obj.bodies{k}.r_Parent + obj.bodies{k}.joint.r_rel);
                else
                    obj.bodies{k}.R_0k = obj.bodies{k}.joint.R_pe;
                    obj.bodies{k}.r_OP = obj.bodies{k}.joint.R_pe.'*(obj.bodies{k}.r_Parent + obj.bodies{k}.joint.r_rel);
                end
                % Determine absolute position of COG
                obj.bodies{k}.r_OG  = obj.bodies{k}.r_OP + obj.bodies{k}.r_G;
                % Determine absolute position of link's ending position
                obj.bodies{k}.r_OPe = obj.bodies{k}.r_OP + obj.bodies{k}.r_Pe;
                % Determine absolute position of the operational space
                if(~isempty(obj.bodies{k}.op_space))
                    obj.bodies{k}.r_Oy  = obj.bodies{k}.r_OP + obj.bodies{k}.r_y;
                end
            end
            
            % Set S (joint state matrix) and S_dot
            index_dofs = 1;
            obj.S = MatrixOperations.Initialise([6*obj.numLinks,obj.numDofs],is_symbolic);
            obj.S_dot = MatrixOperations.Initialise([6*obj.numLinks,obj.numDofs],is_symbolic);
            for k = 1:obj.numLinks
                obj.S(6*k-5:6*k, index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.S;
                obj.S_dot(6*k-5:6*k, index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.S_dot;
                index_dofs = index_dofs + obj.bodies{k}.joint.numDofs;
            end

            % Set P (relationship with joint propagation)
            obj.P = MatrixOperations.Initialise([6*obj.numLinks,6*obj.numLinks],is_symbolic);
            for k = 1:obj.numLinks
                body_k = obj.bodies{k};
                for a = 1:k
                    body_a = obj.bodies{a};
                    R_ka = body_k.R_0k.'*body_a.R_0k;
                    Pak = obj.bodiesPathGraph(a,k)*[R_ka*body_a.joint.R_pe.' -R_ka*MatrixOperations.SkewSymmetric(-body_a.r_OP + R_ka.'*body_k.r_OG); ...
                        zeros(3,3) R_ka];
                    obj.P(6*k-5:6*k, 6*a-5:6*a) = Pak;
                end
            end

            % W = P*S
            obj.W = obj.P*obj.S;
            % Determine x_dot
            obj.x_dot = obj.W*obj.q_dot;
            % Extract absolute velocities
            for k = 1:obj.numLinks
                obj.bodies{k}.v_OG = obj.x_dot(6*k-5:6*k-3);
                obj.bodies{k}.w = obj.x_dot(6*k-2:6*k);
            end          

            % Determine x_ddot
            ang_mat = MatrixOperations.Initialise([6*obj.numLinks,6*obj.numLinks],is_symbolic);
            for k = 1:obj.numLinks
                kp = obj.bodies{k}.parentLinkId;
                if (kp > 0)
                    w_kp = obj.bodies{kp}.w;
                else
                    w_kp = zeros(3,1);
                end
                w_k = obj.bodies{k}.w;
                ang_mat(6*k-5:6*k, 6*k-5:6*k) = [2*MatrixOperations.SkewSymmetric(w_kp) zeros(3,3); zeros(3,3) MatrixOperations.SkewSymmetric(w_k)];
            end

            obj.C_a = obj.P*obj.S_dot*obj.q_dot + obj.P*ang_mat*obj.S*obj.q_dot;
            for k = 1:obj.numLinks
                for a = 1:k
                    ap = obj.bodies{a}.parentLinkId;
                    if (ap > 0 && obj.bodiesPathGraph(a,k))
                        obj.C_a(6*k-5:6*k-3) = obj.C_a(6*k-5:6*k-3) + obj.bodies{k}.R_0k.'*obj.bodies{ap}.R_0k*cross(obj.bodies{ap}.w, cross(obj.bodies{ap}.w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel));
                    end
                end
                obj.C_a(6*k-5:6*k-3) = obj.C_a(6*k-5:6*k-3) + cross(obj.bodies{k}.w, cross(obj.bodies{k}.w, obj.bodies{k}.r_G));
            end
            obj.x_ddot = obj.P*obj.S*obj.q_ddot + obj.C_a;
            
            % Extract absolute accelerations
            for k = 1:obj.numLinks
                obj.bodies{k}.a_OG = obj.x_ddot(6*k-5:6*k-3);
                obj.bodies{k}.w_dot = obj.x_ddot(6*k-2:6*k);
            end
            
            % The operational space variables
            if(obj.occupied.op_space)
                % Now determine the operational space vector y
                obj.y = MatrixOperations.Initialise([obj.numOPDofs,1],is_symbolic); l = 1;
                for k = 1:obj.numLinks
                    if(~isempty(obj.bodies{k}.op_space))
                        n_y = obj.bodies{k}.numOPDofs;
                        obj.y(l:l+n_y-1) = obj.bodies{k}.op_space.extractOpSpace(obj.bodies{k}.r_Oy,obj.bodies{k}.R_0k);
                        l = l + n_y;
                    end
                end
                
                % Set Q (relationship with joint propagation for operational space)
                Q = MatrixOperations.Initialise([6*obj.numLinks,6*obj.numLinks],is_symbolic);
                for k = 1:obj.numLinks
                    body_k = obj.bodies{k};
                    for a = 1:k
                        body_a = obj.bodies{a};
                        R_ka = body_k.R_0k.'*body_a.R_0k;
                        Qak = [body_k.R_0k,zeros(3);zeros(3),body_k.R_0k]*(obj.bodiesPathGraph(a,k)*[R_ka*body_a.joint.R_pe.' -R_ka*MatrixOperations.SkewSymmetric(-body_a.r_OP + R_ka.'*body_k.r_Oy); ...
                            zeros(3,3) R_ka]);
                        Q(6*k-5:6*k, 6*a-5:6*a) = Qak;
                    end
                end
                % J = T*Q*S
                obj.J = obj.T*Q*obj.S;
                % Determine y_dot
                obj.y_dot = obj.J*obj.q_dot;
                
                % Determine J_dot
                temp_j_dot = Q*obj.S_dot + Q*ang_mat*obj.S;
                for k = 1:obj.numLinks
                    for a = 1:k
                        ap = obj.bodies{a}.parentLinkId;
                        if (ap > 0 && obj.bodiesPathGraph(a,k))
                            temp_j_dot(6*k-5:6*k-3,:) = temp_j_dot(6*k-5:6*k-3,:) - ...
                                obj.bodies{k}.R_0k*obj.bodies{k}.R_0k.'*obj.bodies{ap}.R_0k*MatrixOperations.SkewSymmetric(obj.bodies{ap}.w)*MatrixOperations.SkewSymmetric(obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel)*obj.W(6*ap-2:6*ap,:);
                        end
                    end
                    temp_j_dot(6*k-5:6*k-3,:) = temp_j_dot(6*k-5:6*k-3,:) - obj.bodies{k}.R_0k*MatrixOperations.SkewSymmetric(obj.bodies{k}.w)*MatrixOperations.SkewSymmetric(obj.bodies{k}.r_y)*obj.W(6*k-2:6*k,:);
                end
                obj.J_dot = obj.T*temp_j_dot;
                obj.y_ddot = obj.J_dot*q_dot + obj.J*obj.q_ddot;
            end
            
            % The dynamics variables
            if(obj.occupied.dynamics)
                obj.update_dynamics();
            end
        end
        
        function update_dynamics(obj)
            % Body equation of motion terms
            obj.M_b = obj.massInertiaMatrix*obj.W;
            obj.C_b = obj.massInertiaMatrix*obj.C_a;
            for k = 1:obj.numLinks
                obj.C_b(6*k-2:6*k) = obj.C_b(6*k-2:6*k) + cross(obj.bodies{k}.w, obj.bodies{k}.I_G*obj.bodies{k}.w);
                if(isa(obj.C_b,'sym'))
                    simplify(cross(obj.bodies{k}.w, obj.bodies{k}.I_G*obj.bodies{k}.w))
                end
            end
            obj.G_b = MatrixOperations.Initialise([6*obj.numLinks,1],isa(obj.q,'sym'));
            for k = 1:obj.numLinks
                obj.G_b(6*k-5:6*k-3) = obj.bodies{k}.R_0k.'*[0; 0; -obj.bodies{k}.m*SystemModel.GRAVITY_CONSTANT];
            end
            
            % Joint space equation of motion terms
            obj.M =   obj.W.' * obj.M_b;
            obj.C =   obj.W.' * obj.C_b;
            obj.G = - obj.W.' * obj.G_b;
            
            % Operational space equation of motion terms
            if(obj.occupied.op_space)
                obj.M_y = inv(obj.J*inv(obj.M)*obj.J'); %#ok<MINV>
                Jpinv   = obj.J'/(obj.J*obj.J');
                obj.C_y = Jpinv'*obj.C - obj.M_y*obj.J_dot*obj.q_dot;
                obj.G_y = Jpinv'*obj.G;
            end
        end

        % Supporting function to connect all of the parent and child bodies
        function formConnectiveMap(obj)
            for k = 1:obj.numLinks
                obj.connectBodies(obj.bodies{k}.parentLinkId, k, obj.bodies{k}.r_Parent);
            end
        end

        % Supporting function to connect a particular child to a parent
        function connectBodies(obj, parent_link_num, child_link_num, r_parent_loc)
            assert(parent_link_num < child_link_num, 'Parent link number must be smaller than child');
            assert(~isempty(obj.bodies{child_link_num}), 'Child link does not exist');
            if parent_link_num > 0
                assert(~isempty(obj.bodies{parent_link_num}), 'Parent link does not exist');
            end

            obj.bodiesPathGraph(child_link_num, child_link_num) = 1;
            child_link = obj.bodies{child_link_num};
            if parent_link_num == 0
                parent_link = [];
            else
                parent_link = obj.bodies{parent_link_num};
            end
            child_link.addParent(parent_link, r_parent_loc);
            obj.connectivityGraph(parent_link_num+1, child_link_num) = 1;

            if (parent_link_num > 0)
                obj.bodiesPathGraph(parent_link_num, child_link_num) = 1;
                obj.bodiesPathGraph(:, child_link_num) = obj.bodiesPathGraph(:, child_link_num) | obj.bodiesPathGraph(:, parent_link_num);
            end
        end
        
        function q = qIntegrate(obj, q0, q_dot, dt)
            index_vars = 1;
            q = zeros(size(q0));
            for k = 1:obj.numLinks
                q(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.QIntegrate(q0, q_dot, dt);
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end
        
        function q = get.q_default(obj)
            q = zeros(obj.numDofVars, 1);
            index_vars = 1;
            for k = 1:obj.numLinks
                q(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_default;
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end

        function q_dot = get.q_dot_default(obj)
            q_dot = zeros(obj.numDofs, 1);
            index_dofs = 1;
            for k = 1:obj.numLinks
                q_dot(index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.q_dot_default;
                index_dofs = index_dofs + obj.bodies{k}.joint.numDofs;
            end
        end

        function q_ddot = get.q_ddot_default(obj)
            q_ddot = zeros(obj.numDofs, 1);
            index_dofs = 1;
            for k = 1:obj.numLinks
                q_ddot(index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1) = obj.bodies{k}.joint.q_ddot_default;
                index_dofs = index_dofs + obj.bodies{k}.joint.numDofs;
            end
        end
        
        function q_lb = get.q_lb(obj)
            q_lb = zeros(obj.numDofVars, 1);
            index_vars = 1;
            for k = 1:obj.numLinks
                q_lb(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_lb;
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end
        
        function q_ub = get.q_ub(obj)
            q_ub = zeros(obj.numDofVars, 1);
            index_vars = 1;
            for k = 1:obj.numLinks
                q_ub(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_ub;
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end
        
        function q_deriv = get.q_deriv(obj)
            q_deriv = zeros(obj.numDofVars, 1);
            index_vars = 1;
            for k = 1:obj.numLinks
                q_deriv(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_deriv;
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end  
        
        function createMassInertiaMatrix(obj)
            obj.massInertiaMatrix = zeros(6*obj.numLinks, 6*obj.numLinks);
            for k = 1:obj.numLinks
                obj.massInertiaMatrix(6*k-5:6*k, 6*k-5:6*k) = [obj.bodies{k}.m*eye(3) zeros(3,3); zeros(3,3) obj.bodies{k}.I_G];
            end
        end
        
        function M_y = get.M_y(obj)
            if(~obj.occupied.dynamics)
                assert(obj.occupied.op_space, 'Operational space coordinates has not been attached');
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            M_y = obj.M_y;
        end
        
        function C_y = get.C_y(obj)
            if(~obj.occupied.dynamics)
                assert(obj.occupied.op_space, 'Operational space coordinates has not been attached');
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            C_y = obj.C_y;
        end
        
        function G_y = get.G_y(obj)
            if(~obj.occupied.dynamics)
                assert(obj.occupied.op_space, 'Operational space coordinates has not been attached');
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            G_y = obj.G_y;
        end
        
        function M_b = get.M_b(obj)
            if(~obj.occupied.dynamics)
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            M_b = obj.M_b;
        end
        
        function C_b = get.C_b(obj)
            if(~obj.occupied.dynamics)
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            C_b = obj.C_b;
        end
        
        function G_b = get.G_b(obj)
            if(~obj.occupied.dynamics)
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            G_b = obj.G_b;
        end
        
        function M = get.M(obj)
            if(~obj.occupied.dynamics)
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            M = obj.M;
        end
        
        function C = get.C(obj)
            if(~obj.occupied.dynamics)
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            C = obj.C;
        end
        
        function G = get.G(obj)
            if(~obj.occupied.dynamics)
                obj.createMassInertiaMatrix();
                obj.occupied.dynamics = true;
                obj.update_dynamics();
            end
            G = obj.G;
        end
        
        function N = calculate_N(obj)
            % Initialisiation
            flag = isa(obj.q,'sym');
            MassInertiaMatrix = obj.massInertiaMatrix;
            % PS_dot term
            n_q = length(bodyKinematics.q_dot); n_l = bodyKinematics.numLinks;
            N_j = MatrixOperations.Initialise([n_q,n_q^2],flag);
            A = MatrixOperations.Initialise([6*n_l,n_q],flag);
            offset_x = 1; offset_y = 1;
            for i=1:n_l
                [N_jt,A_t] = bodyKinematics.bodies{i}.joint.QuadMatrix(obj.q);
                n_qt = size(N_jt,1);
                N_j(offset_x:offset_x+n_qt-1,(i-1)*n_q+offset_y:(i-1)*n_q+offset_y+n_qt^2-1) = N_jt;
                A((i-1)*6+1:(i-1)*6+6,offset_x:offset_x+n_qt-1) = A_t;
                offset_x = offset_x + n_qt; offset_y = offset_y + n_qt^2;
            end
            % Project correctly
            C1 = MatrixOperations.MatrixProdLeftQuad(obj.W.'*MassInertiaMatrix*obj.P*A,N_j);
            
            % P_dotS term
            C2 = MatrixOperations.Initialise([n_q,n_q^2],flag);
            T_t = zeros(6*n_l,3);
            T_r = zeros(6*n_l,3);
            for i=1:n_l
                ip = bodyKinematics.bodies{i}.parentLinkId;
                if (ip > 0)
                    W_ip = obj.W(6*(ip-1)+4:6*(ip-1)+6,:);
                else
                    W_ip = zeros(3,n_q);
                end
                W_i     =   obj.W(6*(i-1)+4:6*(i-1)+6,:);
                N_t     =   MatrixOperations.GenerateMatrixQuadCross(2*W_ip,obj.S(6*(i-1)+1:6*(i-1)+3,:));
                N_r     =   MatrixOperations.GenerateMatrixQuadCross(W_i,obj.S(6*(i-1)+4:6*(i-1)+6,:));
                T_t(6*(i-1)+1:6*(i-1)+3,:) = eye(3);
                T_r(6*(i-1)+4:6*(i-1)+6,:) = eye(3);
                C2 = C2 + MatrixOperations.MatrixProdLeftQuad(obj.W.'*MassInertiaMatrix*obj.P*T_t,N_t) + MatrixOperations.MatrixProdLeftQuad(obj.W.'*MassInertiaMatrix*obj.P*T_r,N_r);
                T_t(6*(i-1)+1:6*(i-1)+3,:) = zeros(3);
                T_r(6*(i-1)+4:6*(i-1)+6,:) = zeros(3);
            end
            
            % \omega \times \omega \times r
            C3 = MatrixOperations.Initialise([n_q,n_q^2],flag);
            for i = 1:n_l
                N_t = MatrixOperations.Initialise([n_q,3*n_q],flag);
                for j = 1:i
                    jp = bodyKinematics.bodies{j}.parentLinkId;
                    if (jp > 0 && bodyKinematics.bodiesPathGraph(j,i))
                        W_jp = obj.W(6*(jp-1)+4:6*(jp-1)+6,:);
                        R = MatrixOperations.SkewSymmetric(bodyKinematics.bodies{j}.r_Parent + bodyKinematics.bodies{j}.joint.r_rel)*W_jp;
                        N_tt = -MatrixOperations.GenerateMatrixQuadCross(W_jp,R);
                        N_t = N_t + MatrixOperations.MatrixProdLeftQuad(bodyKinematics.bodies{i}.R_0k.'*bodyKinematics.bodies{jp}.R_0k,N_tt);
                    end
                end
                W_i = obj.W(6*(i-1)+4:6*(i-1)+6,:);
                R = MatrixOperations.SkewSymmetric(bodyKinematics.bodies{1}.r_G)*W_i;
                N_tt = -MatrixOperations.GenerateMatrixQuadCross(W_i,R);
                N_t = N_t + N_tt;
                T_t(6*(i-1)+1:6*(i-1)+3,:) = eye(3);
                C3 = C3 + MatrixOperations.MatrixProdLeftQuad(obj.W.'*MassInertiaMatrix*T_t,N_t);
                T_t(6*(i-1)+1:6*(i-1)+3,:) = zeros(3);
            end
            
            % \omega \times I_G \omega
            C4 = MatrixOperations.Initialise([n_q,n_q^2],flag);
            for i = 1:n_l
                W_i = obj.W(6*(i-1)+4:6*(i-1)+6,:);
                N_r = MatrixOperations.GenerateMatrixQuadCross(W_i,obj.bodies{i}.I_G*W_i);
                T_r(6*(i-1)+4:6*(i-1)+6,:) = eye(3);
                C4 = C4 + MatrixOperations.MatrixProdLeftQuad(obj.W.'*T_r,N_r);
                T_r(6*(i-1)+4:6*(i-1)+6,:) = zeros(3);
            end
            % Compute N
            N = C1 + C2 + C3 + C4;
            if(flag)
                N = simplify(N);
                V = MatrixOperations.Initialise([n_q,1],flag);
                for i=1:n_q
                    V(i) = bodyKinematics.q_dot.'*N(:,(i-1)*n_q+1:(i-1)*n_q+n_q)*bodyKinematics.q_dot;
                end
                simplify(V)
            end
        end
        
        function loadOpXmlObj(obj,op_space_xmlobj)
            obj.occupied.op_space = true;
            % Load the op space
            assert(strcmp(op_space_xmlobj.getNodeName, 'op_set'), 'Root element should be <op_set>');
            % Go into the cable set
            allOPItems = op_space_xmlobj.getChildNodes;
            
            num_ops = allOPItems.getLength;
            % Creates all of the operational spaces first first
            for k = 1:num_ops
                % Java uses 0 indexing
                currentOPItem = allOPItems.item(k-1);

                type = char(currentOPItem.getNodeName);
                if (strcmp(type, 'position'))
                    op_space = OpPosition.LoadXmlObj(currentOPItem);
                elseif(strcmp(type, 'orientation_euler_xyz'))
                    op_space = OpOrientationEulerXYZ.LoadXmlObj(currentOPItem);
                elseif(strcmp(type, 'pose_euler_xyz'))
                    op_space = OpPoseEulerXYZ.LoadXmlObj(currentOPItem);
                else
                    error('Unknown link type: %s', type);
                end
                parent_link = op_space.link;
                obj.bodies{parent_link}.attachOPSpace(op_space);
                % Should add some protection to ensure that one OP_Space
                % per link
            end
            num_op_dofs = 0;
            for k = 1:length(obj.bodies)
                num_op_dofs = num_op_dofs + obj.bodies{k}.numOPDofs;
            end
            obj.numOPDofs = num_op_dofs;
            
            obj.T = MatrixOperations.Initialise([obj.numOPDofs,6*obj.numLinks],0);
            l = 1; 
            for k = 1:length(obj.bodies)
                if(~isempty(obj.bodies{k}.op_space))
                    n_y = obj.bodies{k}.numOPDofs;
                    obj.T(l:l+n_y-1,6*k-5:6*k) = obj.bodies{k}.op_space.getSelectionMatrix();
                    l = l + n_y;
                end
            end
        end
    end

    methods (Static)
        function b = LoadXmlObj(body_prop_xmlobj)
            % Load the body
            assert(strcmp(body_prop_xmlobj.getNodeName, 'links'), 'Root element should be <links>');
            
%             allLinkItems = body_prop_xmlobj.getChildNodes;
            allLinkItems = body_prop_xmlobj.getElementsByTagName('link_rigid');

            num_links = allLinkItems.getLength;
            links = cell(1,num_links);

            % Creates all of the links first
            for k = 1:num_links
                % Java uses 0 indexing
                currentLinkItem = allLinkItems.item(k-1);

                num_k = str2double(currentLinkItem.getAttribute('num'));
                assert(num_k == k, sprintf('Link number does not correspond to its order, order: %d, specified num: %d ', k, num_k));

                type = char(currentLinkItem.getNodeName);
                if (strcmp(type, 'link_rigid'))
                    links{k} = BodyModelRigid.LoadXmlObj(currentLinkItem);
                else
                    error('Unknown link type: %s', type);
                end
            end

            % Create the actual object to return
            b = SystemModelBodies(links);
        end
    end
end
