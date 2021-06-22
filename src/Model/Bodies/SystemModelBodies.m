% System kinematics of the bodies for the system
%
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102-1113,
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
        
        % Degrees of freedom
        numDofs
        numDofVars
        numOperationalSpaces = 0
        numOperationalDofs
        numLinks
        numDofsActuated             % Number of actuated DoFs

        % Generalised coordinates of the system
        q                       % Joint space coordinates
        q_dot                   % Derivatives of joint space coordinates

        % Graphs
        connectivityGraph       % p x p connectivity matrix, if (i,j) = 1 means link i-1 is the parent of link j
        bodiesPathGraph         % p x p matrix that governs how to track to particular bodies, (i,j) = 1 means that to get to link j we must pass through link i
                   
        % Kinematics
        R_0ks                   % Matrix storing rotation matrices of bodies (^0_kR)
        r_OPs                   % Matrix storing joint locations (in local frame)
        r_Gs                    % Matrix storing location of CoG (in local frame)
        r_Pes                   % Matrix storing location of end of bodies (in local frame)

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

        W_grad          = []; % The gradient tensor of the W matrix
        
        % Gradient terms
        Minv_grad       = []; % The gradient tensor of the M^-1 matrix
        C_grad_q        = []; % The gradient (wrt q) of C
        C_grad_qdot     = []; % The gradient (wrt \dot{q}) of C
        G_grad          = []; % The gradient tensor of the W matrix

        % Absolute CoM velocities and accelerations (linear and angular)
        x_dot                   % Absolute velocities
        x_ddot                  % Absolute accelerations (x_ddot = W(q)*q_ddot + C_a(q,q_dot))
        C_a                     % Relationship between body and joint accelerations \ddot{\mathbf{x}} = W \ddot{\mathbf{q}} + C_a
        
        % Mass matrix
        massInertiaMatrix = [];       % Mass-inertia 6p x 6p matrix


        % M_b * q_ddot + C_b = G_b + w_b - V^T f (forces in body space)
        M_b = [];                        % Body mass-inertia matrix
        C_b = [];                        % Body C matrix
        G_b = [];                        % Body G matrix

        % M*q_ddot + C + G + w_e = - L^T f (forces in joint space)
        M = [];
        C = [];
        G = [];
        W_e = [];
        A = [];
        
        %
        % Operational space related variables
        %
        % Operational Space coordinates of the system
        y       = [];           % Operational space coordinates
        y_dot   = [];           % Operational space velocity
        y_ddot  = [];           % Operational space acceleration
        % Operational Space Jacobians
        J     = [];             % J matrix representing relationship between generalised coordinate velocities and operational space coordinates
        J_dot = [];             % Derivative of J
        T     = [];             % Projection of operational coordinates 
        % M_y y_ddot + C_y + G_y = W (operational space EoM)
        M_y = [];
        C_y = [];
        G_y = [];
        
        update;                             % Function handle for update function

        % Indices for ease of computation
        qIndexInit                          % A vector consisting of the first index in q to each joint
        operationalSpaceBodyIndices = [];   % A vector containing the list of bodies that operational spaces are attached to
        yIndexInit                          % A vector consisting of the first index in y to each operational space

        % Flags
        modelMode                           % The model mode
        modelOptions                        % ModelOptions object storing options for the model
        
        % Function handles for compiled update mode        
        compiled_R_0ks_fn;
        compiled_r_OPs_fn;
        compiled_P_fn;
        compiled_P_fns;
        compiled_S_fn;
        compiled_S_dot_fn;
        compiled_x_ddot_fn;
        compiled_x_dot_fn;
        
        compiled_C_b_fn;
        compiled_G_b_fn;
        compiled_M_b_fn;
        
        compiled_J_fn;
        compiled_J_dot_fn;
        compiled_y_fn;
        compiled_y_dot_fn;
        compiled_y_ddot_fn;
    end
    
    properties (Access = private)
        compiled_bodies_lib_name;
        compiled_operational_space_lib_name;
    end

    properties (Dependent)
        q_initial
        q_default
        q_dot_default
        q_ddot_default
        q_lb                        % The lower bound on joints that are physically meaningful (by definition)
        q_ub                        % The upper bound on joints that are physically meaningful (by definition)
        q_min                       % Minimum joint values of q (set by user)
        q_max                       % Maximum joint values of q (set by user)
        % Generalised coordinates time derivative (for special cases q_dot does not equal q_deriv)
        q_deriv                     % This is useful particularly if the derivative of q is not the same as q_dot, but in most cases they are the same
        q_dofType                   % The type of DoF (translation or rotation) for each q value
        
        tau                         % The joint actuator
        tauMin                      % The joint actuator minimum value
        tauMax                      % The joint actuator maximum value
        TAU_INVALID
        
        % Get array of dofs for each joint
        jointsNumDofVars
        jointsNumDofs
        % and for each operational space
        operationalSpaceNumDofs
        
        isComputeOperationalSpace
        isSymbolic
    end

    methods
        % Constructor for the class SystemModelBodies.  This determines the
        % numbers of degrees of freedom as well as initialises the
        % matrices.
        function b = SystemModelBodies(bodies, model_mode, model_options, bodies_lib_name, opset_lib_name)
            num_dofs = 0;
            num_dof_vars = 0;
            num_operational_dofs = 0;
            num_dof_actuated = 0;
            b.qIndexInit = zeros(1, b.numLinks);
            for k = 1:length(bodies)
                b.qIndexInit(k) = num_dofs + 1;
                num_dofs = num_dofs + bodies{k}.numDofs;
                num_dof_vars = num_dof_vars + bodies{k}.numDofVars;
                if (bodies{k}.joint.isActuated)
                    num_dof_actuated = num_dof_actuated + bodies{k}.joint.numDofs;
                end
            end
            b.modelMode = model_mode;
            b.modelOptions = model_options;
            b.bodies = bodies;
            b.numDofs = num_dofs;
            b.numDofVars = num_dof_vars;
            b.numOperationalDofs = num_operational_dofs;
            b.numDofsActuated = num_dof_actuated;
            b.numLinks = length(b.bodies);
            
            b.connectivityGraph = MatrixOperations.Initialise([b.numLinks, b.numLinks], false);
            b.bodiesPathGraph = MatrixOperations.Initialise([b.numLinks, b.numLinks], false);
            b.S = MatrixOperations.Initialise([6*b.numLinks, b.numDofs], b.isSymbolic);
            b.S_dot = MatrixOperations.Initialise([6*b.numLinks,b.numDofs], b.isSymbolic);
            b.P = MatrixOperations.Initialise([6*b.numLinks, 6*b.numLinks], b.isSymbolic);
            b.W = MatrixOperations.Initialise([6*b.numLinks, b.numDofs], b.isSymbolic);
            b.T = MatrixOperations.Initialise([0,6*b.numLinks], b.isSymbolic); % TODO
            
            b.R_0ks = MatrixOperations.Initialise([3, 3*b.numLinks], b.isSymbolic);
            b.r_OPs = MatrixOperations.Initialise([3, b.numLinks], b.isSymbolic);
            b.r_Gs  = MatrixOperations.Initialise([3, b.numLinks], false);
            b.r_Pes = MatrixOperations.Initialise([3, b.numLinks], false);
            
            % Construct joint actuation selection matrix
            b.A = zeros(b.numDofs, b.numDofsActuated);
            dof_ind = 0;
            dof_tau = 0;
            for k = 1:b.numLinks
                if (b.bodies{k}.isJointActuated)
                    b.A(dof_ind+1:dof_ind+b.bodies{k}.numDofs, dof_tau+1:dof_tau+b.bodies{k}.numDofs) = eye(b.bodies{k}.numDofs, b.bodies{k}.numDofs);
                    dof_tau = dof_tau + b.bodies{k}.numDofs;
                end
                dof_ind = dof_ind + b.bodies{k}.numDofs;
            end
            
            % Connects the objects of the system and create the
            % connectivity and body path graphs
            b.formConnectiveMap();
            b.createMassInertiaMatrix(); 
            b.set_update_fns(); % Sets the modelMode property and update function handle
            
            
            if (model_mode == ModelModeType.COMPILED)
                if (nargin < 4 || isempty(bodies_lib_name))
                    CASPR_log.Error('Library name must be supplied with COMPILED mode');
                else 
                    b.compiled_bodies_lib_name = bodies_lib_name;
                    b.compiled_operational_space_lib_name = opset_lib_name;
                    
                    b.compiled_R_0ks_fn = str2func([b.compiled_bodies_lib_name, '_compiled_R_0ks']);
                    b.compiled_r_OPs_fn = str2func([b.compiled_bodies_lib_name, '_compiled_r_OPs']);
                    b.compiled_P_fn = str2func([b.compiled_bodies_lib_name, '_compiled_P']);
                    b.compiled_P_fns = cell(b.numLinks, b.numLinks);
                    for k = 1:b.numLinks
                        for a = 1:k
                            b.compiled_P_fns{k,a} = str2func(sprintf('%s_compiled_P_%d_%d', b.compiled_bodies_lib_name, k, a));
                        end
                    end
                    b.compiled_S_fn = str2func([b.compiled_bodies_lib_name, '_compiled_S']);
                    b.compiled_S_dot_fn = str2func([b.compiled_bodies_lib_name, '_compiled_S_dot']);
                    b.compiled_x_ddot_fn = str2func([b.compiled_bodies_lib_name, '_compiled_x_ddot']);
                    b.compiled_x_dot_fn = str2func([b.compiled_bodies_lib_name, '_compiled_x_dot']);
                    
                    b.compiled_C_b_fn = str2func([b.compiled_bodies_lib_name, '_compiled_C_b']);
                    b.compiled_G_b_fn = str2func([b.compiled_bodies_lib_name, '_compiled_G_b']);
                    b.compiled_M_b_fn = str2func([b.compiled_bodies_lib_name, '_compiled_M_b']);
                end
            end
        end

        % Update the kinematics of the body model for the entire
        % system using the generalised coordinates, velocity and
        % acceleration. 3 update functions are set for the 3 model modes.
        % These update function should also be called to
        % update the entire system, rather than calling the update function
        % for each body directly.  
        
        % Update function under DEFAULT 
        % - update using numerical methods
        function defaultUpdate(obj, q, q_dot, q_ddot, w_ext)        
            is_symbolic = obj.isSymbolic; 
            if (is_symbolic)
                CASPR_log.Assert(isa(q, 'sym'), 'q must be symbolic in symbolic mode');
                CASPR_log.Assert(isa(q_dot, 'sym'), 'q_dot must be symbolic in symbolic mode');
                CASPR_log.Assert(isa(q_ddot, 'sym'), 'q_ddot must be symbolic in symbolic mode');
                CASPR_log.Assert(isa(w_ext, 'sym'), 'w_ext must be symbolic in symbolic mode');
            end
            
            num_links = obj.numLinks;
            
            % Assign q, q_dot, q_ddot
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            obj.W_e = w_ext;
                       
            % Update each body first
            index_vars = 1;
            index_dofs = 1;
            for k = 1:num_links
                q_k = q(index_vars:index_vars+obj.bodies{k}.joint.numVars-1);
                q_dot_k = q_dot(index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1);
                q_ddot_k = q_ddot(index_dofs:index_dofs+obj.bodies{k}.joint.numDofs-1);
                obj.bodies{k}.update(q_k, q_dot_k, q_ddot_k);
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
                index_dofs = index_dofs + obj.bodies{k}.joint.numDofs;
            end

            % Now the global system updates
            % Set bodies kinematics (rotation matrices)
            for k = 1:num_links
                body_k = obj.bodies{k};
                parent_link_num = body_k.parentLinkId;
                CASPR_log.Assert(parent_link_num < k, 'Problem with numbering of links with parent and child');

                % Determine rotation matrix
                % Determine joint location
                R_pe = body_k.joint.R_pe;   
                if (is_symbolic && isa(R_pe, 'sym'))
                    R_pe = simplify(R_pe, 'Step', 15);
                end  
                                
                if parent_link_num > 0
                    parent_R_0k = obj.bodies{parent_link_num}.R_0k;
                    body_k.R_0k = parent_R_0k*R_pe; 
                    body_k.r_OP = R_pe.'*(obj.bodies{parent_link_num}.r_OP + body_k.r_Parent + body_k.joint.r_rel);
                else
                    body_k.R_0k = R_pe;
                    body_k.r_OP = R_pe.'*(obj.bodies{k}.r_Parent + body_k.joint.r_rel);
                end
                
                if (is_symbolic)
                    if (isa(body_k.R_0k, 'sym'))
                        CASPR_log.Info(sprintf('- Symbolic simplifying of R_0k for body %d', k));
                        body_k.R_0k = simplify(body_k.R_0k, 'Step', k*15);
                    end
                    CASPR_log.Info(sprintf('- Symbolic simplifying of r_OP for body %d', k));
                    body_k.r_OP = simplify(body_k.r_OP, 'Step', k*15);
                end
                
                % Determine absolute position of COG
                body_k.r_OG  = body_k.r_OP + obj.bodies{k}.r_G;
                % Determine absolute position of link's ending position
                body_k.r_OPe = body_k.r_OP + obj.bodies{k}.r_Pe;
                
                % Determine absolute position of the operational space
                if(~isempty(obj.bodies{k}.operationalSpace))
                    body_k.r_OY  = body_k.r_OP + body_k.r_y;
                end
                % Store information in matrices
                obj.R_0ks(:,3*k-2:3*k) = body_k.R_0k;
                obj.r_OPs(:,k) = body_k.r_OP;
                obj.r_Gs(:,k) = body_k.r_G;
                obj.r_Pes(:,k) = body_k.r_Pe;
            end

            if (is_symbolic)
                CASPR_log.Info('Symbolic computing of S and S_dot');
            end
            % Set S (joint state matrix) and S_dot            
            index_dofs = 1;
            for k = 1:num_links
                body_k = obj.bodies{k};
                obj.S(6*k-5:6*k, index_dofs:index_dofs+body_k.joint.numDofs-1) = body_k.joint.S;
                obj.S_dot(6*k-5:6*k, index_dofs:index_dofs+body_k.joint.numDofs-1) = body_k.joint.S_dot;
                index_dofs = index_dofs + body_k.joint.numDofs;
            end            
            
            if (is_symbolic)
                CASPR_log.Info('Symbolic computing/simplifying of P');
            end
            % P
            for k = 1:num_links
                body_k = obj.bodies{k};
                k_R_0k = body_k.R_0k;
                for a = 1:k
                    body_a = obj.bodies{a}; 
                    a_R_0k = body_a.R_0k;         
                    R_ka = k_R_0k.'*a_R_0k;
                    
                    if (is_symbolic)
                        CASPR_log.Info(sprintf('- Symbolic computing of P_ak, k: %d, a: %d', k, a));
                        if (isa(R_ka, 'sym'))
                            CASPR_log.Info(sprintf('- Symbolic simplifying of R_ka, k: %d, a: %d', k, a));
                            R_ka = simplify(R_ka, 'Step', k*15);
                        end
                    end
                    Pak_1 = R_ka*body_a.joint.R_pe.';
                    r_OP = body_a.r_OP;
                    r_OG = body_k.r_OG;
                    Pak_2_1 = MatrixOperations.SkewSymmetric(-r_OP + R_ka.'*r_OG);
                    
                    if (is_symbolic)
                        Pak_2_1 = simplify(Pak_2_1, 'Step', k*10);
                    end
                    
                    Pak_2 = -R_ka*Pak_2_1;
                                        
                    if (is_symbolic)
                        CASPR_log.Info(sprintf('- Symbolic simplifying of P_ak, k: %d, a: %d', k, a));
                        if (isa(Pak_1, 'sym'))
                            Pak_1 = simplify(Pak_1, 'Step', k*5);
                        end
                        Pak_2 = simplify(Pak_2, 'Step', k*5);
                    end
                    
                    Pak = obj.bodiesPathGraph(a,k)*[Pak_1 Pak_2; ...
                        zeros(3,3) R_ka];       
                    obj.P(6*k-5:6*k, 6*a-5:6*a) = Pak; 
                end
            end
            
            if (is_symbolic)
                CASPR_log.Info('Symbolic computing of W');
            end
            % W = P*S              
            obj.W = obj.P*obj.S;       
                   
            % Determine x_dot              
            obj.x_dot = obj.W*obj.q_dot;          
            
            % Extract absolute velocities
            for k = 1:num_links
                obj.bodies{k}.v_OG = obj.x_dot(6*k-5:6*k-3);
                obj.bodies{k}.w = obj.x_dot(6*k-2:6*k);
            end
            
            if (is_symbolic)
                CASPR_log.Info('Symbolic computing/simplifying of C_a');
            end
            % C_a
            ang_mat = MatrixOperations.Initialise([6*num_links,6*num_links], is_symbolic);
            for k = 1:num_links
                kp = obj.bodies{k}.parentLinkId;
                if (kp > 0)
                    w_kp = obj.bodies{kp}.w;
                else
                    w_kp = zeros(3,1);
                end
                w_k = obj.bodies{k}.w;   
                    
                if (is_symbolic)
                    w_k = simplify(w_k);
                end
                    
                ang_mat(6*k-5:6*k, 6*k-5:6*k) = [2*MatrixOperations.SkewSymmetric(w_kp) zeros(3,3); zeros(3,3) MatrixOperations.SkewSymmetric(w_k)];
            end

            C_a_t = obj.P*(obj.S_dot + ang_mat*obj.S)*obj.q_dot;
            for k = 1:num_links
                for a = 1:k
                    ap = obj.bodies{a}.parentLinkId;
                    if (ap > 0 && obj.bodiesPathGraph(a,k))
                        C_a_t(6*k-5:6*k-3) = C_a_t(6*k-5:6*k-3) + obj.bodies{k}.R_0k.'*obj.bodies{ap}.R_0k*cross(obj.bodies{ap}.w, cross(obj.bodies{ap}.w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel));
                    end
                end
                C_a_t(6*k-5:6*k-3) = C_a_t(6*k-5:6*k-3) + cross(obj.bodies{k}.w, cross(obj.bodies{k}.w, obj.bodies{k}.r_G));
            end
            obj.C_a = C_a_t;
            
            obj.x_ddot = obj.W*obj.q_ddot + obj.C_a;
            
            % Extract absolute accelerations
            for k = 1:num_links
                obj.bodies{k}.a_OG = obj.x_ddot(6*k-5:6*k-3);
                obj.bodies{k}.w_dot = obj.x_ddot(6*k-2:6*k);
            end                  
                        
            % The operational space variables
            if (obj.isComputeOperationalSpace)
                obj.update_operational_space();                        
            end            
            
            % The dynamics variables (both joint and operational spaces)
            % Hence must run after the update_operational_space()
            if(obj.modelOptions.isComputeDynamics || obj.modelMode ~= ModelModeType.DEFAULT)
                obj.update_dynamics();
            end
                        
            % The Hessian Variables
            if(obj.modelOptions.isComputeHessian)
                obj.update_hessian();
            end  
        
            % The linearisation variables
            if(obj.modelOptions.isComputeLinearisation)
                obj.update_linearisation();
            end    
        end
        
        
        % Update function under COMPILED mode
        % - Update using compiled files       
        function compiledUpdate(obj, q, q_dot, q_ddot, w_ext)            
            % Update the 4 arguments
            obj.q = q;
            obj.q_dot = q_dot;
            obj.q_ddot = q_ddot;
            obj.W_e = w_ext;
            
            num_links = obj.numLinks;
            
            % Body kinematics
            obj.R_0ks = obj.compiled_R_0ks_fn(q, q_dot, q_ddot, w_ext);
            obj.r_OPs = obj.compiled_r_OPs_fn(q, q_dot, q_ddot, w_ext);
            
            for k = 1:num_links
                body_k = obj.bodies{k};
                obj.r_Gs(:,k) = body_k.r_G;
                obj.r_Pes(:,k) = body_k.r_Pe;
            end
            
%             for k = 1:obj.numLinks
%                 obj.bodies{k}.R_0k = obj.R_0ks(:,3*k-2:3*k);
%                 obj.bodies{k}.r_OP = obj.r_OPs(:,k);
%                 obj.bodies{k}.r_OG = obj.r_OPs(:,k) + obj.r_Gs(:,k);
%                 obj.bodies{k}.r_OPe = obj.r_OPs(:,k) + obj.r_Pes(:,k);
%             end
            
            % Kinematics jacobians
            obj.P = obj.compiled_P_fn(q, q_dot, q_ddot, w_ext);            
            obj.S = obj.compiled_S_fn(q, q_dot, q_ddot, w_ext);
            obj.S_dot = obj.compiled_S_dot_fn(q, q_dot, q_ddot, w_ext);
            
            obj.W = obj.P*obj.S;      
            
            obj.x_ddot = obj.compiled_x_ddot_fn(q, q_dot, q_ddot, w_ext);
            obj.x_dot = obj.compiled_x_dot_fn(q, q_dot, q_ddot, w_ext);
            
            % Update class properties by calling the compiled functions
            if obj.modelOptions.isComputeDynamics
                obj.C_b = obj.compiled_C_b_fn(q, q_dot, q_ddot, w_ext);
                obj.G_b = obj.compiled_G_b_fn(q, q_dot, q_ddot, w_ext);
                obj.M_b = obj.compiled_M_b_fn(q, q_dot, q_ddot, w_ext);
                obj.M =   obj.W.' * obj.M_b;
                obj.C =   obj.W.' * obj.C_b;
                obj.G = - obj.W.' * obj.G_b;
            end
            
            % Update operational space variables
            if (obj.isComputeOperationalSpace)             
                obj.J = obj.compiled_J_fn(q, q_dot, q_ddot, w_ext);
                obj.J_dot = obj.compiled_J_dot_fn(q, q_dot, q_ddot, w_ext);
                obj.y = obj.compiled_y_fn(q, q_dot, q_ddot, w_ext);
                obj.y_dot = obj.compiled_y_dot_fn(q, q_dot, q_ddot, w_ext);       
                obj.y_ddot = obj.compiled_y_ddot_fn(q, q_dot, q_ddot, w_ext);         
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
            CASPR_log.Assert(parent_link_num < child_link_num, 'Parent link number must be smaller than child');
            CASPR_log.Assert(~isempty(obj.bodies{child_link_num}), 'Child link does not exist');
            if parent_link_num > 0
                CASPR_log.Assert(~isempty(obj.bodies{parent_link_num}), 'Parent link does not exist');
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

        % A function to integrate the joint space.
        function q = qIntegrate(obj, q0, q_dot, dt)
            index_vars = 1;
            q = zeros(size(q0));
            for k = 1:obj.numLinks
                q(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.QIntegrate(q0(index_vars:index_vars+obj.bodies{k}.joint.numVars-1), q_dot(index_vars:index_vars+obj.bodies{k}.joint.numVars-1), dt);
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end
        
        % Create the mass inertia matrix for the system from the joint mass
        % inertia matrices.
        function createMassInertiaMatrix(obj)
            obj.massInertiaMatrix = zeros(6*obj.numLinks, 6*obj.numLinks);
            for k = 1:obj.numLinks
                obj.massInertiaMatrix(6*k-5:6*k, 6*k-5:6*k) = [obj.bodies{k}.m*eye(3) zeros(3,3); zeros(3,3) obj.bodies{k}.I_G];
            end
        end
        
        % Updates the inertia properties and the mass inertia matrix
        function updateInertiaProperties(obj,m,r_G,I_G)
            % Ensure that the inputs are cell arrays when necessary
            CASPR_log.Assert((isempty(m)||(obj.numLinks==1)||iscell(m))&&(isempty(r_G)||(obj.numLinks==1)||iscell(r_G))&&(isempty(I_G)||(obj.numLinks==1)||iscell(I_G)),'Input must be a cell if the number of links is greater than 1');
            % Ensure that the non empty cell array is of the correct length
            CASPR_log.Assert((isempty(m)||(length(m)==obj.numLinks))&&(isempty(r_G)||(length(r_G)==obj.numLinks))&&(isempty(I_G)||(length(I_G)==obj.numLinks)),'Inertia terms must be empty or a cell array of size equal to the number of links');
            
            % Go through the cell array of each element
            for i = 1:length(m)
                obj.bodies{i}.m = m{i};
            end
            for i = 1:length(r_G)
                obj.bodies{i}.r_G = r_G{i};
            end
            for i = 1:length(I_G)
                obj.bodies{i}.I_G = I_G{i};
            end
            % Update the mass inertia matrix to reflect the new values
            obj.createMassInertiaMatrix();
            obj.update_dynamics();
        end

        % calculate the quadratic form of centrifugal/Coriolis term
        function N = quadraticC(obj)
            n_l = obj.numLinks;
            n_q = length(obj.q_dot);
            N = zeros(n_q,n_q,n_q);
            vq = zeros(6,n_q);
            aq = zeros(6,n_q,n_q);
            fJ = cell(n_l,1);
            
            n_q_0_to_i = 0;
            for i = 1:n_l
                n_q_i = obj.bodies{i}.joint.numDofs;
                r_i_const = obj.bodies{i}.r_Parent;
                % this is doubtful, not sure if the transpose should exist
                % I think this should be correct, R_pe represents rotation
                % matrix that converts a vector coordinate in the rotated
                % frame into a coordinate in the original frame
                R_i = obj.bodies{i}.joint.R_pe';
                % r_i is the joint induced translation, represented in the
                % link frame
                r_i = obj.bodies{i}.joint.r_rel;
                r_CoM_i = obj.bodies{i}.r_G;
                I_CoM_i = obj.bodies{i}.I_G;
                m_i = obj.bodies{i}.m;
                S_i = [obj.bodies{i}.joint.S(4:6,:); obj.bodies{i}.joint.S(1:3,:)];
                S_grad_i = [obj.bodies{i}.joint.S_grad(4:6,:,:); obj.bodies{i}.joint.S_grad(1:3,:,:)];
                XM_i_from_parent = xltMMat(r_i)*rotMat(R_i)*xltMMat(r_i_const);
                Is_i = spatialInertiaTensor(I_CoM_i, r_CoM_i, m_i);
                
                % coordinate transformatioin
                for j = 1:n_q_0_to_i
                    vq(:,j) = XM_i_from_parent*vq(:,j);
                    for k = 1:n_q_0_to_i
                        aq(:,j,k) = XM_i_from_parent*aq(:,j,k);
                    end
                end
                
                % update joint velocity/acceleration - add joint i's contribution
                for j = 1:n_q_i
                    vq(:,n_q_0_to_i+j) = S_i(:,j);
                end
                for j = 1:n_q_i
                    for k = 1:n_q_0_to_i+n_q_i
                        aq(:,n_q_0_to_i+j,k) = vecX6DM(vq(:,k))*S_i(:,j);
                    end
                    for k = 1:n_q_i
                        aq(:,n_q_0_to_i+j,n_q_0_to_i+k) = aq(:,n_q_0_to_i+j,n_q_0_to_i+k) + S_grad_i(:,j,k);
                    end
                end
                
                n_q_0_to_i = n_q_0_to_i + n_q_i;
                fJ{i} = zeros(6,n_q,n_q);
                % compute joint force
                for j = 1:n_q_0_to_i
                    for k = 1:n_q_0_to_i
                        fJ{i}(:,j,k) = Is_i*aq(:,j,k) + vecX6DF(vq(:,k))*Is_i*vq(:,j);
                    end
                end
            end
            
            q_index = n_q;
            for cnt = 1:n_l
                i = n_l-cnt+1;
                n_q_i = obj.bodies{i}.joint.numDofs;
                S_i = [obj.bodies{i}.joint.S(4:6,:); obj.bodies{i}.joint.S(1:3,:)];
                % derive quadratic C for joint i
                for j = 1:n_q_i
                    tmp_index = q_index - n_q_i + j;
                    for m = 1:n_q
                        for n = 1:n_q
                            N(m,n,tmp_index) = S_i(:,j)'*fJ{i}(:,m,n);
                        end
                    end
                end
                % update fJ
                if i-1 ~= 0
                    r_i_const = obj.bodies{i}.r_Parent;
                    % this is doubtful, not sure if the transpose should exist
                    % I think this should be correct, R_pe represents rotation
                    % matrix that converts a vector coordinate in the rotated
                    % frame into a coordinate in the original frame
                    R_i = obj.bodies{i}.joint.R_pe';
                    % r_i is the joint induced translation, represented in the
                    % link frame
                    r_i = obj.bodies{i}.joint.r_rel;
                    XF_i_to_parent = xltFMat(-r_i_const)*rotMat(R_i')*xltFMat(-r_i);
                    for m = 1:n_q
                        for n = 1:n_q
                            fJ{i-1}(:,m,n)=fJ{i-1}(:,m,n)+XF_i_to_parent*fJ{i}(:,m,n);
                        end
                    end
                end
                q_index = q_index - n_q_i;
            end
            
        end
                
        % Calculate the internal matrices for the quadratic form of the
        % Coriolis/Centrifugal forces.
        function N = calculateN(obj)
            % Initialisiation
            flag = isa(obj.q,'sym');
            MassInertiaMatrix = obj.massInertiaMatrix;
            % PS_dot term
            n_q = length(obj.q_dot); n_l = obj.numLinks;
            N_j = MatrixOperations.Initialise([n_q,n_q^2],flag);
            A_t = MatrixOperations.Initialise([6*n_l,n_q],flag);
            offset_x = 1; offset_y = 1;
            for i=1:n_l
                [N_jt,A_t] = obj.bodies{i}.joint.QuadMatrix(obj.q);
                n_qt = size(N_jt,1);
                N_j(offset_x:offset_x+n_qt-1,(i-1)*n_q+offset_y:(i-1)*n_q+offset_y+n_qt^2-1) = N_jt;
                A_t((i-1)*6+1:(i-1)*6+6,offset_x:offset_x+n_qt-1) = A_t;
                offset_x = offset_x + n_qt; offset_y = offset_y + n_qt^2;
            end
            % Project correctly
            C1 = MatrixOperations.MatrixProdLeftQuad(obj.W.'*MassInertiaMatrix*obj.P*A_t,N_j);

            % P_dotS term
            C2 = MatrixOperations.Initialise([n_q,n_q^2],flag);
            T_t = zeros(6*n_l,3);
            T_r = zeros(6*n_l,3);
            for i=1:n_l
                ip = obj.bodies{i}.parentLinkId;
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
                    jp = obj.bodies{j}.parentLinkId;
                    if (jp > 0 && obj.bodiesPathGraph(j,i))
                        W_jp = obj.W(6*(jp-1)+4:6*(jp-1)+6,:);
                        R = MatrixOperations.SkewSymmetric(obj.bodies{j}.r_Parent + obj.bodies{j}.joint.r_rel)*W_jp;
                        N_tt = -MatrixOperations.GenerateMatrixQuadCross(W_jp,R);
                        N_t = N_t + MatrixOperations.MatrixProdLeftQuad(obj.bodies{i}.R_0k.'*obj.bodies{jp}.R_0k,N_tt);
                    end
                end
                W_i = obj.W(6*(i-1)+4:6*(i-1)+6,:);
                R = MatrixOperations.SkewSymmetric(obj.bodies{1}.r_G)*W_i;
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
                    V(i) = obj.q_dot.'*N(:,(i-1)*n_q+1:(i-1)*n_q+n_q)*obj.q_dot;
                end
                simplify(V)
            end
        end

        % Load the operational space xml object
        function load_operational_space_xml_obj(obj, operational_space_xmlobj)
            % Load the op space
            CASPR_log.Assert(strcmp(operational_space_xmlobj.getNodeName, 'operational_set'), 'Root element should be <operational_set>');
            % Go into the operational space set
            allOperationalItems = operational_space_xmlobj.getChildNodes;

            num_operationals = allOperationalItems.getLength;
            obj.operationalSpaceBodyIndices = zeros(1, num_operationals);
            obj.yIndexInit = zeros(1, num_operationals);
            
            num_operational_dofs = 0;
            
            % Creates all of the operational spaces first first
            for k = 1:num_operationals
                % Java uses 0 indexing
                currentOperationalItem = allOperationalItems.item(k-1);
                
                type = char(currentOperationalItem.getNodeName);
                if (strcmp(type, 'position'))
                    operational_space = OperationalPosition.LoadXmlObj(currentOperationalItem);
                elseif(strcmp(type, 'orientation_euler_xyz'))
                    operational_space = OperationalOrientationEulerXYZ.LoadXmlObj(currentOperationalItem);
                elseif(strcmp(type, 'pose_euler_xyz'))
                    operational_space = OperationalPoseEulerXYZ.LoadXmlObj(currentOperationalItem);
                else
                    CASPR_log.Printf(sprintf('Unknown operational space type: %s', type),CASPRLogLevel.ERROR);
                end
                parent_link = operational_space.link;
                obj.operationalSpaceBodyIndices(k) = parent_link;
                obj.bodies{parent_link}.attachOperationalSpace(operational_space);
                
                obj.yIndexInit = num_operational_dofs + 1;
                num_operational_dofs = num_operational_dofs + operational_space.numOperationalDofs;
            end            
            obj.numOperationalDofs = num_operational_dofs;
            obj.numOperationalSpaces = num_operationals;

            obj.T = MatrixOperations.Initialise([obj.numOperationalDofs,6*obj.numLinks],0);
            l = 1;
            for k = 1:length(obj.operationalSpaceBodyIndices)
                index = obj.operationalSpaceBodyIndices(k);
                n_y = obj.bodies{index}.numOperationalDofs;
                obj.T(l:l+n_y-1,6*index-5:6*index) = obj.bodies{index}.operationalSpace.getSelectionMatrix();
                l = l + n_y;
            end
            
            if (obj.modelMode == ModelModeType.COMPILED)
                obj.compiled_J_fn = str2func([obj.compiled_operational_space_lib_name, '_compiled_J']);
                obj.compiled_J_dot_fn = str2func([obj.compiled_operational_space_lib_name, '_compiled_J_dot']);
                obj.compiled_y_fn = str2func([obj.compiled_operational_space_lib_name, '_compiled_y']);
                obj.compiled_y_dot_fn = str2func([obj.compiled_operational_space_lib_name, '_compiled_y_dot']);
                obj.compiled_y_ddot_fn = str2func([obj.compiled_operational_space_lib_name, '_compiled_y_ddot']);
            end
        end

        % -------
        % Getters
        % -------        
        function q = get.q_initial(obj)            
            q = zeros(obj.numDofVars, 1);
            index_vars = 1;
            for k = 1:obj.numLinks
                q(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_initial;
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

        function q_min = get.q_min(obj)            
            q_min = zeros(obj.numDofVars, 1);
            index_vars = 1;
            for k = 1:obj.numLinks
                q_min(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_min;
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end

        function q_max = get.q_max(obj)          
            q_max = zeros(obj.numDofVars, 1);
            index_vars = 1;
            for k = 1:obj.numLinks
                q_max(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_max;
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end
        
        function q_deriv = get.q_deriv(obj)
            if obj.modelMode==ModelModeType.COMPILED
                q_deriv = obj.q_dot;
            else
                q_deriv = zeros(obj.numDofVars, 1);
                index_vars = 1;
                for k = 1:obj.numLinks
                    q_deriv(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_deriv;
                    index_vars = index_vars + obj.bodies{k}.joint.numVars;
                end
            end            
        end
        
        function q_dofType = get.q_dofType(obj)           
            index_vars = 1;
            for k = 1:obj.numLinks
                q_dofType(index_vars:index_vars+obj.bodies{k}.joint.numVars-1) = obj.bodies{k}.joint.q_dofType;
                index_vars = index_vars + obj.bodies{k}.joint.numVars;
            end
        end
                
        function set.tau(obj, value)
            CASPR_log.Assert(length(value) == obj.numDofsActuated, 'Cannot set tau since the value does not match the actuated DoFs');
            count = 0;
            for k = 1:obj.numLinks
                if (obj.bodies{k}.joint.isActuated)
                    num_dofs = obj.bodies{k}.joint.numDofs;
                    obj.bodies{k}.joint.set_tau(value(count+1:count+num_dofs));
                    count = count + num_dofs;
                end
            end
        end
        
        function val = get.tau(obj)            
            val = zeros(obj.numDofsActuated, 1);
            count = 0;
            for k = 1:obj.numLinks
                if (obj.bodies{k}.joint.isActuated)
                    num_dofs = obj.bodies{k}.joint.numDofs;
                    val(count+1:count+num_dofs) = obj.bodies{k}.joint.tau;
                    count = count + num_dofs;
                end
            end
        end
        
        function val = get.tauMin(obj)            
            val = zeros(obj.numDofsActuated, 1);
            count = 0;
            for k = 1:obj.numLinks
                if (obj.bodies{k}.joint.isActuated)
                    num_dofs = obj.bodies{k}.joint.numDofs;
                    val(count+1:count+num_dofs) = obj.bodies{k}.joint.tau_min;
                    count = count + num_dofs;
                end
            end
        end
        
        function set.tauMin(obj,val)
            if length(val) == obj.numDofsActuated
                count = 0;
                for k = 1:obj.numLinks
                    if (obj.bodies{k}.joint.isActuated)
                        num_dofs = obj.bodies{k}.joint.numDofs;
                        obj.bodies{k}.joint.set_tau_min(val(count+1:count+num_dofs));
                        count = count + num_dofs;
                    end
                end
            else
                % invalid input, do nothing
                CASPR_log.Warn('The given minimum joint torque is invalid, abort update.');
            end
        end
        
        function val = get.tauMax(obj)            
            val = zeros(obj.numDofsActuated, 1);
            count = 0;
            for k = 1:obj.numLinks
                if (obj.bodies{k}.joint.isActuated)
                    num_dofs = obj.bodies{k}.joint.numDofs;
                    val(count+1:count+num_dofs) = obj.bodies{k}.joint.tau_max;
                    count = count + num_dofs;
                end
            end
        end
        
        function set.tauMax(obj,val)
            if length(val) == obj.numDofsActuated
                count = 0;
                for k = 1:obj.numLinks
                    if (obj.bodies{k}.joint.isActuated)
                        num_dofs = obj.bodies{k}.joint.numDofs;
                        obj.bodies{k}.joint.set_tau_max(val(count+1:count+num_dofs));
                        count = count + num_dofs;
                    end
                end
            else
                % invalid input, do nothing
                CASPR_log.Warn('The given maximum joint torque is invalid, abort update.');
            end
        end
        
        function value = get.TAU_INVALID(obj)           
            value = JointBase.INVALID_TAU * ones(obj.numDofsActuated, 1);
        end
        
        function jointsNumDofVars = get.jointsNumDofVars(obj)            
            jointsNumDofVars = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                jointsNumDofVars(k) = obj.bodies{k}.numDofVars;
            end
        end
        
        function jointsNumDofs = get.jointsNumDofs(obj)          
            jointsNumDofs = zeros(obj.numLinks,1);
            for k = 1:obj.numLinks
                jointsNumDofs(k) = obj.bodies{k}.numDofs;
            end
        end
               
        function operationalNumDofs = get.operationalSpaceNumDofs(obj)          
            operationalNumDofs = zeros(obj.numOperationalSpaces,1);
            for k = 1:obj.numOperationalSpaces
                operationalNumDofs(k) = obj.bodies{obj.operationalSpaceBodyIndices(k)}.operationalSpace.numOperationalDofs;
            end
        end
        
        function value = get.isComputeOperationalSpace(obj)
            value = (obj.numOperationalSpaces > 0);
        end
        
        function value = get.isSymbolic(obj)
            value = (obj.modelMode == ModelModeType.SYMBOLIC);
        end
        
        function value = get.W_grad(obj)
            if obj.modelMode==ModelModeType.COMPILED || ~obj.modelOptions.isComputeHessian
                %CASPR_log.Warn('W_grad is not computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = obj.W_grad;
            end
        end
        
        % Compiling body variables under COMPILED mode
        % - Symbolic Variables are compiled into .m files and saved to the
        % path
        function compileBodies(obj, path, lib_name)
            CASPR_log.Assert(obj.modelMode == ModelModeType.SYMBOLIC, 'Can only compile a symbolic model');
            % Jacobians
            CASPR_log.Info('Compiling S and S_dot...');
            matlabFunction(obj.S, 'File', strcat(path, '/', lib_name, '_compiled_S'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
            matlabFunction(obj.S_dot, 'File', strcat(path, '/', lib_name, '_compiled_S_dot'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
            
            % Kinematics
            CASPR_log.Info('Compiling R_0ks...');
            matlabFunction(obj.R_0ks, 'File', strcat(path, '/', lib_name, '_compiled_R_0ks'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
                
            CASPR_log.Info('Compiling r_OPs...');
            matlabFunction(obj.r_OPs, 'File', strcat(path, '/', lib_name, '_compiled_r_OPs'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
            
            % Absolute CoM velocities and accelerations (linear and angular)
            CASPR_log.Info('Compiling Velocities and Accelerations...');        
            CASPR_log.Info('- Compiling x_dot...');            
            matlabFunction(obj.x_dot, 'File', strcat(path, '/', lib_name, '_compiled_x_dot'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
            CASPR_log.Info('- Compiling x_ddot...');            
            matlabFunction(obj.x_ddot, 'File', strcat(path, '/', lib_name, '_compiled_x_ddot'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
            
            % Dynamics
            %if(obj.modelOptions.isComputeDynamics)
                CASPR_log.Info('Compiling Dynamics Variables...'); 
                CASPR_log.Info('- Compiling M_b...'); 
                matlabFunction(obj.M_b, 'File', strcat(path, '/', lib_name, '_compiled_M_b'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
                CASPR_log.Info('- Compiling C_b...'); 
                matlabFunction(obj.C_b, 'File', strcat(path, '/', lib_name, '_compiled_C_b'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
                CASPR_log.Info('- Compiling G_b...'); 
                matlabFunction(obj.G_b, 'File', strcat(path, '/', lib_name, '_compiled_G_b'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});  
            
            CASPR_log.Info('Compiling P...');
            tmp_body_P_path = strcat(path, '/P');
            if ~exist(tmp_body_P_path, 'dir')
                mkdir(tmp_body_P_path);              
            end
            for k = 1:obj.numLinks
                for a = 1:k
                    CASPR_log.Info(sprintf('- Compiling P%d%d...', k, a));
                    P_temp = obj.P(6*k-5:6*k, 6*a-5:6*a);
                    matlabFunction(P_temp, 'File', sprintf('%s/%s_compiled_P_%d_%d', tmp_body_P_path, lib_name, k, a), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
                end
            end
            
            % Write a loop to construct the compiled_P file
            % It calls the individual sub-P functions
            compiled_P_fn_name = [lib_name, '_compiled_P'];
            fid = fopen(strcat(path, '/', compiled_P_fn_name, '.m'), 'w');
            fprintf(fid, 'function out1 = %s(in1,in2,in3,in4)\n', compiled_P_fn_name);
            fprintf(fid, 'out1 = zeros(%d,%d);\n', 6*obj.numLinks, 6*obj.numLinks);
            for k = 1:obj.numLinks
                for a = 1:k
                    fprintf(fid, 'out1(%d:%d, %d:%d) = %s_%d_%d(in1,in2,in3,in4);\n', 6*k-5, 6*k, 6*a-5, 6*a, compiled_P_fn_name, k, a);
                end
            end
            fclose(fid);
%                 matlabFunction(obj.M, 'File', strcat(path, '/', lib_name, '_compiled_M'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
%                 matlabFunction(obj.C, 'File', strcat(path, '/', lib_name, '_compiled_C'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
%                 matlabFunction(obj.G, 'File', strcat(path, '/', lib_name, '_compiled_G'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
%                 matlabFunction(inv(obj.M), 'File', strcat(path, '/', lib_name, '_compiled_Minv'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e}); 
            %end            
            
        end
        
        function compileOperationalSpace(obj, path, lib_name)
            CASPR_log.Assert(obj.modelMode == ModelModeType.SYMBOLIC, 'Can only compile a symbolic model');
            % Operational space
            %if(obj.isComputeOperationalSpace)
                CASPR_log.Info('- Compiling Operational Space Variables...');
                matlabFunction(obj.y, 'File', strcat(path, '/', lib_name, '_compiled_y'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});                
                matlabFunction(obj.y_dot, 'File', strcat(path, '/', lib_name, '_compiled_y_dot'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});                
                matlabFunction(obj.y_ddot, 'File', strcat(path, '/', lib_name, '_compiled_y_ddot'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});                
                matlabFunction(obj.J, 'File', strcat(path, '/', lib_name, '_compiled_J'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
                matlabFunction(obj.J_dot, 'File', strcat(path, '/', lib_name, '_compiled_J_dot'), 'Vars', {obj.q, obj.q_dot, obj.q_ddot, obj.W_e});
            %end
        end
    end

    methods (Access = private)
        % Update the operational space variables
        function update_operational_space(obj)
            is_symbolic = obj.modelMode == ModelModeType.SYMBOLIC; 
            
            obj.y = MatrixOperations.Initialise([obj.numOperationalDofs,1], is_symbolic);
            l = 1;
            for k = 1:length(obj.operationalSpaceBodyIndices)
                body_index = obj.operationalSpaceBodyIndices(k);
                n_y = obj.bodies{body_index}.numOperationalDofs;
                R_0k = obj.bodies{body_index}.R_0k;
                if is_symbolic
                    R_0k = simplify(R_0k, 'Step', k*50);
                end
                obj.bodies{body_index}.r_OY  = obj.bodies{body_index}.r_OP + obj.bodies{body_index}.r_y;
                obj.y(l:l+n_y-1) = obj.bodies{body_index}.operationalSpace.extractOperationalSpace(obj.bodies{body_index}.r_OY, R_0k);
                l = l + n_y;
            end            
            
            if is_symbolic
                obj.y = simplify(obj.y);
            end
            
            % ang_mat
            ang_mat = MatrixOperations.Initialise([6*obj.numLinks,6*obj.numLinks], is_symbolic);
            for k = 1:obj.numLinks
                kp = obj.bodies{k}.parentLinkId;
                if (kp > 0)
                    w_kp = obj.bodies{kp}.w;
                else
                    w_kp = zeros(3,1);
                end
                w_k = obj.bodies{k}.w;
                if is_symbolic
                    w_k = simplify(w_k);
                end
                ang_mat(6*k-5:6*k, 6*k-5:6*k) = [2*MatrixOperations.SkewSymmetric(w_kp) zeros(3,3); zeros(3,3) MatrixOperations.SkewSymmetric(w_k)];
            end

            % Set Q (relationship with joint propagation for operational space)
            if is_symbolic
                CASPR_log.Info('Symbolic computing/simplifying of Q');
            end
            Q = MatrixOperations.Initialise([6*obj.numLinks,6*obj.numLinks], is_symbolic);
            for k = 1:obj.numLinks
                body_k = obj.bodies{k};
                k_R_0k = body_k.R_0k;
                if (is_symbolic)
                    k_R_0k = simplify(k_R_0k, 'Step', k*50);
                end
                for a = 1:k
                    body_a = obj.bodies{a};
                    a_R_0k = body_a.R_0k;
                    if (is_symbolic)
                        a_R_0k = simplify(a_R_0k, 'Step', k*20);
                        R_ka = k_R_0k.'*a_R_0k;
                        R_ka = simplify(R_ka, 'Step', k*20);
                    else
                        R_ka = k_R_0k.'*a_R_0k;
                    end
                    Qak = [k_R_0k,zeros(3);zeros(3),k_R_0k]*(obj.bodiesPathGraph(a,k)*[R_ka*body_a.joint.R_pe.' -R_ka*MatrixOperations.SkewSymmetric(-body_a.r_OP + R_ka.'*body_k.r_OY); ...
                        zeros(3,3) R_ka]);
                    Q(6*k-5:6*k, 6*a-5:6*a) = Qak;
                end
            end
            if is_symbolic
                Q = simplify(Q);
            end

            % J = T*Q*S
            if is_symbolic
                CASPR_log.Info('Symbolic computing/simplifying of J');
            end
            obj.J = obj.T*Q*obj.S;
            
            % Determine y_dot
            if is_symbolic
                CASPR_log.Info('Symbolic computing/simplifying of y_dot');
            end
            obj.y_dot = obj.J*obj.q_dot;
            
            % Determine J_dot
            if is_symbolic
                CASPR_log.Info('Symbolic computing/simplifying of J_dot');
            end
            temp_j_dot = Q*obj.S_dot + Q*ang_mat*obj.S;
            for k = 1:obj.numLinks
                k_R_0k = obj.bodies{k}.R_0k;
                if (is_symbolic)
                    k_R_0k = simplify(k_R_0k, 'Step', k*50);
                end
                for a = 1:k
                    ap = obj.bodies{a}.parentLinkId;
                    if (ap > 0 && obj.bodiesPathGraph(a,k))
                        temp_j_dot(6*k-5:6*k-3,:) = temp_j_dot(6*k-5:6*k-3,:) - ...
                            k_R_0k*k_R_0k.'*obj.bodies{ap}.R_0k*MatrixOperations.SkewSymmetric(obj.bodies{ap}.w)*MatrixOperations.SkewSymmetric(obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel)*obj.W(6*ap-2:6*ap,:);
                    end
                end
                temp_j_dot(6*k-5:6*k-3,:) = temp_j_dot(6*k-5:6*k-3,:) - obj.bodies{k}.R_0k*MatrixOperations.SkewSymmetric(obj.bodies{k}.w)*MatrixOperations.SkewSymmetric(obj.bodies{k}.r_y)*obj.W(6*k-2:6*k,:);
            end
            obj.J_dot = obj.T*temp_j_dot;
            if (is_symbolic)
                obj.J_dot = simplify(obj.J_dot);
            end

            % Determine y_ddot
            if is_symbolic
                CASPR_log.Info('Symbolic computing/simplifying of y_ddot');
            end
            obj.y_ddot = obj.J_dot*obj.q_dot + obj.J*obj.q_ddot;
        end

        % Update the dynamics of the body model for the entire
        % system using the generalised coordinates, velocity and
        % acceleration. This update function is only called when a dynamics
        % object has been used.
        function update_dynamics(obj)        
            is_symbolic = obj.isSymbolic; 
            if (is_symbolic)
                CASPR_log.Info('Symbolic computing M_b, C_b, G_b terms');
            end
            % Body equation of motion terms           
            obj.M_b = obj.massInertiaMatrix*obj.W;
            obj.C_b = obj.massInertiaMatrix*obj.C_a;
            obj.G_b = MatrixOperations.Initialise([6*obj.numLinks,1], obj.isSymbolic);
            for k = 1:obj.numLinks
                body_k = obj.bodies{k};
                R_0k = body_k.R_0k;
                obj.C_b(6*k-2:6*k) = obj.C_b(6*k-2:6*k) + cross(body_k.w, body_k.I_G*body_k.w);
                obj.G_b(6*k-5:6*k-3) = R_0k.'*[0; 0; -body_k.m*SystemModel.GRAVITY_CONSTANT];
            end  
            
            if is_symbolic
%                 CASPR_log.Info('- Symbolic simplifying M_b...');
%                 obj.M_b = simplify(obj.M_b);
%                 CASPR_log.Info('- Symbolic simplifying C_b...');
%                 obj.C_b = simplify(obj.C_b);
%                 CASPR_log.Info('- Symbolic simplifying G_b...');
%                 obj.G_b = simplify(obj.G_b);
            end
           
            if (is_symbolic)
                CASPR_log.Info('Symbolic computing of M, C, G terms');
            end
            % Joint space equation of motion terms             
            obj.M =   obj.W.' * obj.M_b;
            obj.C =   obj.W.' * obj.C_b;
            obj.G = - obj.W.' * obj.G_b;    
                
            % Operational space equation of motion terms
            % It takes 'forever' to calculate the symbolic M_y...
            if(obj.isComputeOperationalSpace && ~obj.isSymbolic)
                obj.M_y = inv(obj.J*inv(obj.M)*obj.J'); %#ok<MINV>
                Jpinv   = obj.J'/(obj.J*obj.J');
                obj.C_y = Jpinv'*obj.C - obj.M_y*obj.J_dot*obj.q_dot;
                obj.G_y = Jpinv'*obj.G;
            end
        end        
        
        % Update the hessian of the body model for the entire
        % system using the generalised coordinates, velocity and
        % acceleration. This update function is only called when a hessian
        % object has been used.
        function update_hessian(obj)
            % This function computes the tensor W_grad = P_grad*S +
            % S_grad*P
            % In the interest of saving memory blockwise computation will
            % be used in place of storing complete gradient tensors       
           
            obj.W_grad = MatrixOperations.Initialise([6*obj.numLinks,obj.numDofs,obj.numDofs], obj.isSymbolic);
            % At the moment I will separate the two loops
            for k = 1:obj.numLinks
                index_a_dofs = 1;
                for a = 1:k
                    S_a_grad = obj.bodies{a}.joint.S_grad;
                    body_dofs = obj.bodies{a}.joint.numDofs;
                    P_ka = obj.P(6*k-5:6*k, 6*a-5:6*a);
                    % S_Grad component
                    % Add P \nabla S
                    obj.W_grad(6*k-5:6*k,index_a_dofs:index_a_dofs+body_dofs-1,index_a_dofs:index_a_dofs+body_dofs-1) = obj.W_grad(6*k-5:6*k,index_a_dofs:index_a_dofs+body_dofs-1,index_a_dofs:index_a_dofs+body_dofs-1) + TensorOperations.LeftMatrixProduct(P_ka,S_a_grad, obj.isSymbolic);
                    % P_grad component
                    P_ka_grad = obj.compute_Pka_grad(k,a);
                    % Add \nabla P S
                    obj.W_grad(6*k-5:6*k,index_a_dofs:index_a_dofs+body_dofs-1,1:obj.numDofs) = obj.W_grad(6*k-5:6*k,index_a_dofs:index_a_dofs+body_dofs-1,1:obj.numDofs) + TensorOperations.RightMatrixProduct(P_ka_grad,obj.S(6*a-5:6*a, index_a_dofs:index_a_dofs+body_dofs-1), obj.isSymbolic);
                    index_a_dofs = index_a_dofs + body_dofs;
                end
            end                      
        end

        % Update the linearisation of the body model for the entire
        % system using the generalised coordinates, velocity and
        % acceleration. This update function is only called when a
        % linearisation object has been used.
        function update_linearisation(obj)
            CASPR_log.Assert(obj.modelOptions.isComputeHessian, 'Cannot update linearisation without hessian mode on');
                
            % Store the transpose gradient as this will be reused.
            W_t_grad = TensorOperations.Transpose(obj.W_grad,[1,2], obj.isSymbolic);
            % M_grad
            Minv = inv(obj.M);
            temp_tensor =  TensorOperations.RightMatrixProduct(W_t_grad,(obj.massInertiaMatrix*obj.W), obj.isSymbolic) + ...
                TensorOperations.LeftMatrixProduct((obj.W.'*obj.massInertiaMatrix),obj.W_grad, obj.isSymbolic);
            obj.Minv_grad = -TensorOperations.LeftRightMatrixProduct(Minv,temp_tensor, obj.isSymbolic);
            % C_grad_q - Note C_1 = W'M_BP(\dot{S}+XS)\dot{q} and C_2 = W'c
            ang_mat = MatrixOperations.Initialise([6*obj.numLinks,6*obj.numLinks], obj.isSymbolic);
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
            % First term is not computed as a block
            obj.C_grad_q        =   TensorOperations.VectorProduct(W_t_grad,obj.massInertiaMatrix*obj.P*(obj.S_dot + ang_mat*obj.S)*obj.q_dot,2, obj.isSymbolic);
            WtM                 =   obj.W.'*obj.massInertiaMatrix;
            obj.C_grad_qdot     =   WtM*obj.P*(obj.S_dot + ang_mat*obj.S);
            % Block computation
            for k = 1:obj.numLinks
                index_a_dofs = 1;
                m_k = obj.bodies{k}.m;
                for a = 1:k
                    body_dofs = obj.bodies{a}.joint.numDofs;
                    ap = obj.bodies{a}.parentLinkId;
                    % Derive the original block matrices
                    % X_a
                    X_a = ang_mat(6*a-5:6*a, 6*a-5:6*a);
                    % \dot{S}_a + X_aS_a
                    S_deriv_a = obj.bodies{a}.joint.S_dot + X_a*obj.bodies{a}.joint.S;
                    % P_ka
                    P_ka = obj.P(6*k-5:6*k, 6*a-5:6*a);
                    % Block gradient computation
                    % Grad(P_ka)S_deriv_a\dot{q}_a component
                    P_ka_grad = obj.compute_Pka_grad(k,a);
                    block_grad = TensorOperations.VectorProduct(P_ka_grad,S_deriv_a*obj.q_dot(index_a_dofs:index_a_dofs+body_dofs-1),2, obj.isSymbolic);
                    % P_ka Grad(S_deriv_a)\dot{q}_a component
                    % Grad(S_deriv_a) = Grad(\dot{S}) + Grad(X_a)S + X_a Grad(S)
                    % Initialise the terms
                    S_deriv_grad_q = MatrixOperations.Initialise([6,body_dofs,obj.numDofs], obj.isSymbolic);
                    S_deriv_grad_q_dot = MatrixOperations.Initialise([6,body_dofs,obj.numDofs], obj.isSymbolic);
                    % Add the Grad(\dot{S}) terms
                    S_deriv_grad_q(:,:,index_a_dofs:index_a_dofs+body_dofs-1) = obj.bodies{a}.joint.S_dot_grad;
                    S_deriv_grad_q_dot(:,:,index_a_dofs:index_a_dofs+body_dofs-1) = obj.bodies{a}.joint.S_grad;
                    % Grad(X_a)S
                    X_a_grad_q = MatrixOperations.Initialise([6,6,obj.numDofs], obj.isSymbolic);
                    X_a_grad_q_dot = MatrixOperations.Initialise([6,6,obj.numDofs], obj.isSymbolic);
                    
                    [w_ap_grad_q,w_ap_grad_q_dot]   = obj.generate_omega_grad(ap);
                    [w_a_grad_q,w_a_grad_q_dot]     = obj.generate_omega_grad(a);
                    for i=1:obj.numDofs
                        X_a_grad_q(1:3,1:3,i) = 2*MatrixOperations.SkewSymmetric(w_ap_grad_q(:,i));
                        X_a_grad_q(4:6,4:6,i) = MatrixOperations.SkewSymmetric(w_a_grad_q(:,i));
                        X_a_grad_q_dot(1:3,1:3,i) = 2*MatrixOperations.SkewSymmetric(w_ap_grad_q_dot(:,i));
                        X_a_grad_q_dot(4:6,4:6,i) = MatrixOperations.SkewSymmetric(w_a_grad_q_dot(:,i));
                    end
                    S_deriv_grad_q = S_deriv_grad_q + TensorOperations.RightMatrixProduct(X_a_grad_q,obj.bodies{a}.joint.S, obj.isSymbolic);
                    S_deriv_grad_q_dot = S_deriv_grad_q_dot + TensorOperations.RightMatrixProduct(X_a_grad_q_dot,obj.bodies{a}.joint.S, obj.isSymbolic);
                    % X_a Grad(S)
                    S_deriv_grad_q(:,:,index_a_dofs:index_a_dofs+body_dofs-1) = S_deriv_grad_q(:,:,index_a_dofs:index_a_dofs+body_dofs-1) + TensorOperations.LeftMatrixProduct(X_a,obj.bodies{a}.joint.S_grad, obj.isSymbolic);
                    
                    % Final computation
                    block_grad = block_grad + P_ka*TensorOperations.VectorProduct(S_deriv_grad_q,obj.q_dot(index_a_dofs:index_a_dofs+body_dofs-1),2, obj.isSymbolic);
                    
                    % Map the block gradient back into the relevant term
                    obj.C_grad_q = obj.C_grad_q + WtM(:,6*k-5:6*k)*block_grad;
                    
                    % C_grad_q_dot
                    obj.C_grad_qdot = obj.C_grad_qdot + WtM(:,6*k-5:6*k)*P_ka*TensorOperations.VectorProduct(S_deriv_grad_q_dot,obj.q_dot(index_a_dofs:index_a_dofs+body_dofs-1),2, obj.isSymbolic);
                    
                    % The c term
                    if(ap==0)
                        R_kam1 = obj.bodies{k}.R_0k.';
                        ap_w = zeros(3,1);
                    else
                        R_kam1 = obj.bodies{k}.R_0k.'*obj.bodies{ap}.R_0k;
                        ap_w = obj.bodies{ap}.w;
                    end
                    R_kam1_grad = obj.compute_Rka_grad(k,ap);
                    
                    % Grad(W)*c
                    obj.C_grad_q = obj.C_grad_q + m_k*TensorOperations.VectorProduct(W_t_grad(:,6*k-5:6*k-3,:),R_kam1*cross(ap_w, cross(ap_w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel)),2, obj.isSymbolic);
                    % Grad of R
                    obj.C_grad_q = obj.C_grad_q + m_k*obj.W(6*k-5:6*k-3,:).'*TensorOperations.VectorProduct(R_kam1_grad,cross(ap_w, cross(ap_w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel)),2, obj.isSymbolic);
                    % Grad of omega
                    obj.C_grad_q = obj.C_grad_q + m_k*obj.W(6*k-5:6*k-3,:).'*R_kam1*TensorOperations.VectorProduct(0.5*X_a_grad_q(1:3,1:3,:),cross(ap_w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel),2, obj.isSymbolic);
                    % Grad of omega
                    obj.C_grad_q = obj.C_grad_q + m_k*obj.W(6*k-5:6*k-3,:).'*R_kam1*MatrixOperations.SkewSymmetric(ap_w)*TensorOperations.VectorProduct(0.5*X_a_grad_q(1:3,1:3,:),obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel,2, obj.isSymbolic);
                    % Grad of r
                    obj.C_grad_q(:,index_a_dofs:index_a_dofs+body_dofs-1) = obj.C_grad_q(:,index_a_dofs:index_a_dofs+body_dofs-1) + m_k*obj.W(6*k-5:6*k-3,:).'*R_kam1*MatrixOperations.SkewSymmetric(ap_w)*MatrixOperations.SkewSymmetric(ap_w)*obj.bodies{a}.joint.S(1:3,:);
                    
                    % q_dot
                    obj.C_grad_qdot = obj.C_grad_qdot + m_k*obj.W(6*k-5:6*k-3,:).'*R_kam1*TensorOperations.VectorProduct(0.5*X_a_grad_q_dot(1:3,1:3,:),cross(ap_w, obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel),2, obj.isSymbolic);
                    obj.C_grad_qdot = obj.C_grad_qdot + m_k*obj.W(6*k-5:6*k-3,:).'*R_kam1*MatrixOperations.SkewSymmetric(ap_w)*TensorOperations.VectorProduct(0.5*X_a_grad_q_dot(1:3,1:3,:),obj.bodies{a}.r_Parent + obj.bodies{a}.joint.r_rel,2, obj.isSymbolic);
                    
                    % update a_dofs
                    index_a_dofs = index_a_dofs + body_dofs;
                end
                % This is where the other terms go.  They are pretty much
                % Grad(W)*c
                obj.C_grad_q = obj.C_grad_q + m_k*TensorOperations.VectorProduct(W_t_grad(:,6*k-5:6*k-3,:),cross(obj.bodies{k}.w, cross(obj.bodies{k}.w, obj.bodies{k}.r_G)),2, obj.isSymbolic);
                % Grad of omega
                obj.C_grad_q = obj.C_grad_q + m_k*obj.W(6*k-5:6*k-3,:).'*TensorOperations.VectorProduct(X_a_grad_q(4:6,4:6,:),cross(obj.bodies{k}.w, obj.bodies{k}.r_G),2, obj.isSymbolic);
                % Grad of omega
                obj.C_grad_q = obj.C_grad_q + m_k*obj.W(6*k-5:6*k-3,:).'*MatrixOperations.SkewSymmetric(obj.bodies{k}.w)*TensorOperations.VectorProduct(X_a_grad_q(4:6,4:6,:),obj.bodies{k}.r_G,2, obj.isSymbolic);
                %
                obj.C_grad_q = obj.C_grad_q + TensorOperations.VectorProduct(W_t_grad(:,6*k-2:6*k,:),cross(obj.bodies{k}.w, obj.bodies{k}.I_G*obj.bodies{k}.w),2, obj.isSymbolic);
                obj.C_grad_q = obj.C_grad_q + obj.W(6*k-2:6*k,:).'*TensorOperations.VectorProduct(X_a_grad_q(4:6,4:6,:),obj.bodies{k}.I_G*obj.bodies{k}.w,2, obj.isSymbolic);
                obj.C_grad_q = obj.C_grad_q + obj.W(6*k-2:6*k,:).'*MatrixOperations.SkewSymmetric(obj.bodies{k}.w)*obj.bodies{k}.I_G*w_a_grad_q;
                
                % q_dot
                obj.C_grad_qdot = obj.C_grad_qdot + m_k*obj.W(6*k-5:6*k-3,:).'*TensorOperations.VectorProduct(X_a_grad_q_dot(4:6,4:6,:),cross(obj.bodies{k}.w, obj.bodies{k}.r_G),2, obj.isSymbolic);
                obj.C_grad_qdot = obj.C_grad_qdot + m_k*obj.W(6*k-5:6*k-3,:).'*MatrixOperations.SkewSymmetric(obj.bodies{k}.w)*TensorOperations.VectorProduct(X_a_grad_q_dot(4:6,4:6,:),obj.bodies{k}.r_G,2, obj.isSymbolic);
                %
                obj.C_grad_qdot = obj.C_grad_qdot + obj.W(6*k-2:6*k,:).'*TensorOperations.VectorProduct(X_a_grad_q_dot(4:6,4:6,:),obj.bodies{k}.I_G*obj.bodies{k}.w,2, obj.isSymbolic);
                obj.C_grad_qdot = obj.C_grad_qdot + obj.W(6*k-2:6*k,:).'*MatrixOperations.SkewSymmetric(obj.bodies{k}.w)*obj.bodies{k}.I_G*w_a_grad_q_dot;
            end
            % G_grad
            temp_grad = MatrixOperations.Initialise([6*obj.numLinks,obj.numDofs], obj.isSymbolic);
            for k = 1:obj.numLinks
                R_k0_grad = obj.compute_Rka_grad(k,0);
                temp_grad(6*k-5:6*k-3,:) = TensorOperations.VectorProduct(R_k0_grad,[0; 0; -obj.bodies{k}.m*SystemModel.GRAVITY_CONSTANT],2, obj.isSymbolic);
            end
            obj.G_grad = -TensorOperations.VectorProduct(W_t_grad,obj.G_b,2, obj.isSymbolic) - obj.W.'*temp_grad;
        end
        
        % Generate the internal SKARot matrix for hessian and linearisation
        % computation.
        function S_KA = generate_SKA_rot(obj,k,a)
            if(a==0)
                R_a0 = eye(3);
            else
                R_a0 = obj.bodies{a}.R_0k.';
            end
            S_KA = MatrixOperations.Initialise([3,obj.qIndexInit(k)+obj.bodies{k}.numDofs-1],obj.modelMode == ModelModeType.SYMBOLIC);
            if((k==0)||(a==0)||obj.bodiesPathGraph(a,k)||obj.bodiesPathGraph(k,a))
                % The bodies are connected
                if(a<=k)
                    for i =a+1:k
                        if(obj.bodiesPathGraph(i,k))
                            body_i = obj.bodies{i};
                            S_KA(:,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1) = -R_a0*body_i.R_0k*obj.S(6*i-2:6*i,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1);
                        end
                    end
                else
                    for i =k+1:a
                        if(obj.bodiesPathGraph(i,a))
                            body_i = obj.bodies{i};
                            S_KA(:,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1) = R_a0*body_i.R_0k*obj.S(6*i-2:6*i,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1);
                        end
                    end
                end
            end
        end
        
        % Generate the internal SKACrossRot matrix for hessian and linearisation
        % computation.
        function S_K = generate_SKA_cross_rot(obj,k,a)
            body_a = obj.bodies{a};
            body_k = obj.bodies{k};
            R_a0 = body_a.R_0k.';
            R_k0 = body_k.R_0k.';
            S_K = MatrixOperations.Initialise([3,obj.qIndexInit(k)+obj.bodies{k}.numDofs-1],obj.modelMode == ModelModeType.SYMBOLIC);
            if(obj.bodiesPathGraph(a,k)||obj.bodiesPathGraph(k,a))
                for i = a+1:k
                    if((obj.bodiesPathGraph(i,a))||(obj.bodiesPathGraph(a,i)))
                        body_i = obj.bodies{i};
                        R_0i = body_i.R_0k;
                        S_K(:,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1) = -R_a0*R_0i*MatrixOperations.SkewSymmetric(-body_i.r_OP + (R_k0*R_0i).'*body_k.r_OG)*obj.S(6*i-2:6*i,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1);
                    end
                end
            end
        end

        % Generate the internal SKATrans matrix for hessian and linearisation
        % computation.
        function S_KA = generateSKATrans(obj,k,a)
            CASPR_log.Assert(k>=a,'Invalid input to generateSKATrans')
            R_a0 = obj.bodies{a}.R_0k.';
            S_KA = MatrixOperations.Initialise([3,obj.qIndexInit(k)+obj.bodies{k}.numDofs-1],obj.modelMode == ModelModeType.SYMBOLIC);
            if(obj.bodiesPathGraph(a,k)||obj.bodiesPathGraph(k,a))
                for i =a+1:k
                    if(obj.bodiesPathGraph(i,a)||(obj.bodiesPathGraph(a,i)))
                        body_ip = obj.bodies{obj.bodies{i}.parentLinkId};
                        body_i = obj.bodies{i};
                        S_KA(:,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1) = R_a0*body_ip.R_0k*obj.S(6*i-5:6*i-3,obj.qIndexInit(i):obj.qIndexInit(i)+body_i.numDofs-1);
                    end
                end
            end
        end

        % Generate the gradient of the angular velocity of frame k in frame
        % k.
        function [omega_grad_q,omega_grad_q_dot] = generate_omega_grad(obj,k)
            % This generates the gradient matrix (in terms of q) associated with the
            % absolute velocity
            % Gradient with respect to q
            temp_tensor     =   MatrixOperations.Initialise([3,obj.numDofs,obj.numDofs], obj.isSymbolic);
            if(k~=0)
                body_k = obj.bodies{k};
                for a = 1:k
                    if(obj.bodiesPathGraph(a,k)||(obj.bodiesPathGraph(a,k)))
                        body_a = obj.bodies{a};
                        R_ka    = body_k.R_0k.'*body_a.R_0k;
                        % Grad(R)S
                        R_ka_grad = MatrixOperations.Initialise([3,3,obj.numDofs], obj.isSymbolic);
                        R_ka_grad(:,:,:) = obj.compute_Rka_grad(k,a);
                        temp_tensor(:,obj.qIndexInit(a):obj.qIndexInit(a)+obj.bodies{a}.numDofs-1,:) = temp_tensor(:,obj.qIndexInit(a):obj.qIndexInit(a)+obj.bodies{a}.numDofs-1,:) + ...
                            TensorOperations.RightMatrixProduct(R_ka_grad,body_a.joint.S(4:6,:), obj.isSymbolic);
                        % Grad(R)S
                        temp_tensor(:,obj.qIndexInit(a):obj.qIndexInit(a)+obj.bodies{a}.numDofs-1,obj.qIndexInit(a):obj.qIndexInit(a)+obj.bodies{a}.numDofs-1) = temp_tensor(:,obj.qIndexInit(a):obj.qIndexInit(a)+obj.bodies{a}.numDofs-1,obj.qIndexInit(a):obj.qIndexInit(a)+obj.bodies{a}.numDofs-1) + ...
                            TensorOperations.LeftMatrixProduct(R_ka,body_a.joint.S_grad(4:6,:,:), obj.isSymbolic);
                    end
                end
                omega_grad_q = TensorOperations.VectorProduct(temp_tensor,obj.q_dot, 2, obj.isSymbolic);
                % Double check this
                % Gradient with respect to qdot
                omega_grad_q_dot = MatrixOperations.Initialise([3,obj.numDofs], obj.isSymbolic);
                for i = 1:k
                    if(obj.bodiesPathGraph(i,k)||obj.bodiesPathGraph(k,i))
                        R_ki = obj.bodies{k}.R_0k.'*obj.bodies{i}.R_0k;
                        omega_grad_q_dot(:,obj.qIndexInit(i):obj.qIndexInit(i)+obj.bodies{i}.numDofs-1) = R_ki*obj.S(6*i-2:6*i,obj.qIndexInit(i):obj.qIndexInit(i)+obj.bodies{i}.numDofs-1);
                    end
                end
            else
                omega_grad_q = temp_tensor;
                omega_grad_q_dot = temp_tensor;
            end
        end

        % Compute the gradient of R_ka
        function R_ka_grad = compute_Rka_grad(obj,k,a)
            % Check that this is not needed in practice
            R_ka_grad = MatrixOperations.Initialise([3,3,obj.numDofs],obj.modelMode == ModelModeType.SYMBOLIC);
            if(a==0)
                R_ka    = obj.bodies{k}.R_0k.';
            else
                R_ka    = obj.bodies{k}.R_0k.'*obj.bodies{a}.R_0k;
            end
            S_KAr = obj.generate_SKA_rot(k,a);
            for i = 1:size(S_KAr,2)
                R_ka_grad(:,:,i) = R_ka*MatrixOperations.SkewSymmetric(S_KAr(:,i));
            end
        end

        % Compute the gradient of P_ka
        function P_ka_grad = compute_Pka_grad(obj,k,a)
            P_ka_grad = MatrixOperations.Initialise([6,6,obj.numDofs], obj.isSymbolic);
            if(obj.bodiesPathGraph(a,k))
                % Initiailisation
                body_k = obj.bodies{k};
                body_a = obj.bodies{a};
                ap = body_a.parentLinkId;
                

                % Computation
                % TOP LEFT
                P_ka_grad(1:3,1:3,:) = obj.compute_Rka_grad(k,ap);

                % TOP RIGHT
                % This makes use of the product rule
                % First the rotation gradient term
                R_ka    = body_k.R_0k.'*body_a.R_0k;
                R_ka_grad = obj.compute_Rka_grad(k,a);
                temp_grad = MatrixOperations.Initialise([3,3,obj.numDofs], obj.isSymbolic);
                temp_grad(:,:,:) = R_ka_grad;
                P_ka_grad(1:3,4:6,:) = P_ka_grad(1:3,4:6,:) - TensorOperations.RightMatrixProduct(temp_grad,MatrixOperations.SkewSymmetric(-body_a.r_OP + R_ka.'*body_k.r_OG), obj.isSymbolic);
                    
                % Then the skew symettric matrix gradient term
                % Within this start with the relative translation
                temp_grad = MatrixOperations.Initialise([3,3,obj.numDofs], obj.isSymbolic);
                S_KAt  = obj.generateSKATrans(k,a);
                for i = 1:size(S_KAt,2)
                    temp_grad(:,:,i) = MatrixOperations.SkewSymmetric(S_KAt(:,i));
                end
                % S associated with relative rotation in the cross
                % product
                S_KAc  = obj.generate_SKA_cross_rot(k,a);
                for i = 1:size(S_KAc,2)
                    temp_grad(:,:,i) = temp_grad(:,:,i) + MatrixOperations.SkewSymmetric(S_KAc(:,i));
                end
                P_ka_grad(1:3,4:6,:) = P_ka_grad(1:3,4:6,:) - TensorOperations.LeftMatrixProduct(R_ka,temp_grad, obj.isSymbolic);

                % BOTTOM RIGHT
                P_ka_grad(4:6,4:6,:) = R_ka_grad;
            end
        end
               
        
        % set update function
        function set_update_fns(obj) 
            CASPR_log.Assert(~isempty(obj.modelMode), 'The model mode should never empty');
            if obj.modelMode == ModelModeType.DEFAULT || obj.modelMode == ModelModeType.SYMBOLIC
                obj.update = @obj.defaultUpdate;
            elseif obj.modelMode == ModelModeType.COMPILED
                obj.update = @obj.compiledUpdate;                
            else
                CASPR_log.Error('Model mode does not exist');
            end
        end
    end

    methods (Static)
        % Load the bodies xml object.
        function b = LoadXmlObj(body_prop_xmlobj, op_space_set_xmlobj, model_mode, model_options, bodies_lib_name, opset_lib_name)
            % Load the body
            CASPR_log.Assert(strcmp(body_prop_xmlobj.getNodeName, 'links'), 'Root element should be <links>');

            allLinkItems = body_prop_xmlobj.getElementsByTagName('link_rigid');
            num_links = allLinkItems.getLength;
            links = cell(1,num_links);
            % Creates all of the links first
            for k = 1:num_links
                % Java uses 0 indexing
                currentLinkItem = allLinkItems.item(k-1);
                num_k = str2double(currentLinkItem.getAttribute('num'));
                CASPR_log.Assert(num_k == k, sprintf('Link number does not correspond to its order, order: %d, specified num: %d ', k, num_k));
                type = char(currentLinkItem.getNodeName);
                if (strcmp(type, 'link_rigid'))
                    links{k} = BodyModelRigid.LoadXmlObj(currentLinkItem);
                else
                    CASPRLogLevel.Print(sprintf('Unknown link type: %s', type),CASPRLogLevel.ERROR);
                end
            end
            % Create the actual object to return
            b = SystemModelBodies(links, model_mode, model_options, bodies_lib_name, opset_lib_name);
            if (~isempty(op_space_set_xmlobj))
                b.load_operational_space_xml_obj(op_space_set_xmlobj);
            end
        end
    end
end
