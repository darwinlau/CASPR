% System kinematics and dynamics of the cables for the system
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
% Description   :
%    Data structure that represents the kinematics for the cables of the
% system. It contains the kinematics of the set of cables, encapsulated
% within the CableKinematics object.
%   When the kinematics are updated, the V matrix (l_dot = V x_dot) also
%   gets updated and stored.
classdef SystemModelCables < handle
    properties (SetAccess = private)
        % This matrix should probably be computed as needed (dependent
        % variable), but if it is a commonly used matrix (i.e. accessed
        % multiple times even if the system state does not change) then
        % storing it would be more efficient. However this means that
        % update must be performed through this class' update function and
        % not on the body's update directly. This makes sense since just
        % updating one cable without updating the others would cause
        % inconsistency anyway.
        V                           % Cable Jacobian l_dot = V x_dot
        V_grad = [];                % Derivative of V with respect to q
        cables = {};                % Cell array of CableKinematics object
        lengths                     % Vector of lengths for all of the cables
        
        % Attachment location matrix
        r_OAs                       % 6 x no.of segments matrix
        
        % These vectors contain the indices of each active cable
        % corresponding to their index within the cables vector
        numCables = 0;              % The number of cables
        numCablesActive = 0;        % Number of active cables
        cableIndicesActive = [];    % Index of active cables in cables vector
        numLinks = 0;               % The number of links
        K                           % The matix of cable stiffnesses
        modelMode                   % A flag to indicate the need for symbolic computations
        modelOptions                % ModelOptions object storing options for the model
        update                      % Update function handle
        
        % Function handles for compiled update mode
        compiled_r_OAs_fn
        compiled_lengths_fn
        compiled_V_fn
    end
    
    properties (Access = private)
        compiled_lib_name
    end

    properties (Dependent)
        numSegmentsMax              % Maximum number of segments out of all of the cables
        numSegmentsTotal            % Total number of segments out of all of the cables
        
        % Information about cable lengths
        
        lengthsActive               % Vector of lengths for active cables
        lengthsPassive              % Vector of lengths for passive cables
        % Information about cable forces
        forces                      % Vector of forces for all of the cables
        forcesActive                % Vector of forces for active cables
        forcesActiveMin             % Vector of min forces for active cables
        forcesActiveMax             % Vector of max forces for passive cables
        forcesPassive               % Vector of forces for passive cables
        cableIndicesPassive         % Index of passive cables in cables vector
        numCablesPassive            % Number of passive cables
        % Jacobian matrices
        V_active
        V_passive
        V_grad_active
        V_grad_passive
        % Constant active forces invalid setinal vector
        FORCES_ACTIVE_INVALID       % Constant of vector of invalid forces for active cables
        
        isSymbolic
    end

    methods
        function ck = SystemModelCables(cables, bodiesModel, model_mode, model_options, compiled_lib_name)
            if (model_mode == ModelModeType.COMPILED)
                if (nargin < 5 || isempty(compiled_lib_name))
                    CASPR_log.Error('Library name must be supplied with COMPILED mode');
                else 
                    ck.compiled_lib_name = compiled_lib_name;
                end
            end
            
            ck.modelMode = model_mode;
            ck.cables = cables;
            ck.numCables = length(cables);
            ck.numLinks = bodiesModel.numLinks; 
            ck.modelOptions = model_options;
            
            ck.r_OAs = MatrixOperations.Initialise([6,ck.numSegmentsTotal],ck.isSymbolic);
            ck.V = MatrixOperations.Initialise([ck.numCables,6*ck.numLinks], ck.isSymbolic);
            ck.K = MatrixOperations.Initialise([ck.numCables,ck.numCables], ck.isSymbolic);
            ck.lengths = MatrixOperations.Initialise([ck.numCables,1], ck.isSymbolic);
            ck.V_grad = MatrixOperations.Initialise([ck.numCables,6*ck.numLinks,bodiesModel.numDofs], ck.isSymbolic);
            
            ck.set_update_fns();  % Sets the modelMode property and update function handle
        end

        % Update the kinematics of the cables for the entire system using
        % the body kinematics. 3 update functions are set for the 3 model modes.
        % This update function should also be called
        % to update the entire system, rather than calling the update
        % function for each cable directly.
        
        % Update function under DEFAULT mode
        % - update using numerical methods
        function defaultUpdate(obj, bodyModel)
            if (obj.isSymbolic)
                CASPR_log.Assert(bodyModel.isSymbolic, 'body model must be symbolic in symbolic mode');
            end
            CASPR_log.Assert(bodyModel.numLinks == obj.numLinks, 'Number of links between the cable and body kinematics must be consistent');
            is_symbolic = obj.isSymbolic;
            
            % Set each cable's kinematics (absolute attachment locations
            % and segment vectors) and Determine V
            % Must clear the V matrix every time since it sums from itself
            obj.V = MatrixOperations.Initialise([obj.numCables,6*obj.numLinks], is_symbolic);
            
            obj.numCablesActive = 0;
            segment_count = 1;
            ind_active = 1;
            obj.cableIndicesActive = zeros(obj.numCablesActive, 1);    
            
            if (is_symbolic)
                CASPR_log.Info('Symbolic computing/simplifying of cable kinematics');
            end
            
            for i = 1:obj.numCables
                obj.cables{i}.update(bodyModel);
                cable = obj.cables{i};
                num_cable_segments = cable.numSegments;
                
                for j = 1:num_cable_segments
                    if (is_symbolic)
                        CASPR_log.Info(sprintf('- Symbolic computing/simplifying of kinematics of cable %d segment %d', i, j));
                    end
                    segment = cable.segments{j};
                    % Update V elements only when segments are on different
                    % links
                    if segment.attached_links(1)~=segment.attached_links(2)
                        % Starting point
                        k = segment.attached_links(1);
                        if k > 0
                            V_ijk_T = -bodyModel.bodies{k}.R_0k.'*segment.segmentVector/segment.length;
                            if (is_symbolic)
                                V_ijk_T = simplify(V_ijk_T, 'Step', k*5);
                            end
                            % - Translation
                            obj.V(i, 6*k-5:6*k-3) = obj.V(i, 6*k-5:6*k-3) + V_ijk_T.';
                            % - Orientation
                            obj.V(i, 6*k-2:6*k) = obj.V(i, 6*k-2:6*k) + ...
                                cross(segment.attachments{1}.r_GA, V_ijk_T).';
                        end
                        
                        % Ending point
                        k = segment.attached_links(2);
                        if k > 0
                            V_ijk_T = bodyModel.bodies{k}.R_0k.'*segment.segmentVector/segment.length;
                            if (is_symbolic)
                                V_ijk_T = simplify(V_ijk_T, 'Step', k*5);
                            end
                            % - Translation
                            obj.V(i, 6*k-5:6*k-3) = obj.V(i, 6*k-5:6*k-3) + V_ijk_T.';
                            % - Orientation
                            obj.V(i, 6*k-2:6*k) = obj.V(i, 6*k-2:6*k) + ...
                                cross(segment.attachments{2}.r_GA, V_ijk_T).';
                        end
                    end
                    % Store r_OA
                    obj.r_OAs(:,segment_count) = [segment.attachments{1}.r_OA; segment.attachments{2}.r_OA];
                    if (is_symbolic)
                        obj.r_OAs(:,segment_count) = simplify(obj.r_OAs(:,segment_count), 'Step', 5);
                    end
                    segment_count = segment_count + 1;
                end
                obj.K(i,i) = obj.cables{i}.K;

                if (cable.isActive)
                    obj.numCablesActive = obj.numCablesActive + 1;
                    obj.cableIndicesActive(ind_active) = i;
                    ind_active = ind_active + 1;                 
                end 
                obj.lengths(i) = obj.cables{i}.length;      
            end  
%             if (is_symbolic)
%                 CASPR_log.Info('Symbolic computing/simplifying of V');
%                 obj.V = simplify(obj.V, 'Step', 20);
%             end
            
            
            
            
%             May not be needed since should be merged above (if no bugs)                   
%             for i = 1:obj.numCables
%                 % Active cable
%                 if (obj.cables{i}.isActive)
%                     obj.cableIndicesActive(ind_active) = i;
%                     ind_active = ind_active + 1;                    
%                 end               
%                 obj.lengths(i) = obj.cables{i}.length;                
%             end
            
            if(bodyModel.modelOptions.isComputeHessian)
                obj.update_hessian(bodyModel);
            end
        end
        
        
        % Update function under COMPILED mode
        % - Update using compiled files   
        function compiledUpdate(obj, bodyModel)       
            q = bodyModel.q;
            q_dot = bodyModel.q_dot;
            q_ddot = bodyModel.q_ddot;
            W_e = bodyModel.W_e;
            
            % Attachment locations
            obj.r_OAs = obj.compiled_r_OAs_fn(q, q_dot, q_ddot, W_e);
            num_cables_active = 0;
            obj.numCablesActive = 0;
            segment_count = 1;
            ind_active = 1;
            cable_indices_active = zeros(obj.numCablesActive, 1);
            for i = 1:obj.numCables
                if (obj.cables{i}.isActive)
                    num_cables_active = num_cables_active + 1;
                    cable_indices_active(ind_active) = i;
                    ind_active = ind_active + 1;
                end
            end
            obj.numCablesActive = num_cables_active;
            obj.cableIndicesActive = cable_indices_active;
            % Cable length
            obj.lengths = obj.compiled_lengths_fn(q, q_dot, q_ddot, W_e); 
            % Matrix
            obj.V = obj.compiled_V_fn(q, q_dot, q_ddot, W_e); 
        end

        % Returns the c_{ijk} element of the CRM
        % CRM is m x s x (p+1) matrix representing the cable-routing
        function c_ijk = getCRMTerm(obj, i, j, k)
            CASPR_log.Assert(i <= obj.numCables, 'Invalid cable number.');
            c_ijk = obj.cables{i}.getCRMTerm(j, k);
        end

        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done. This may be useful when automating the design of the cable
        % attachments?
        function clearCableSegments(obj)
            % remove all cables
            for i = 1:obj.numCables
                obj.cables{i}.clearSegments();
            end
        end

        function s_max = get.numSegmentsMax(obj)
            s_max = 0;
            for i = 1:obj.numCables
                s_max = max([s_max obj.cables{i}.numSegments]);
            end
        end
        
        function value = get.numSegmentsTotal(obj)
            value = 0;
            for i = 1:obj.numCables
                value = value + obj.cables{i}.numSegments;
            end
        end

        function value = get.lengths(obj)
            value = obj.lengths;
        end
        
        function value = get.lengthsActive(obj)           
            value = obj.lengths(obj.cableIndicesActive);
        end
        
        function value = get.lengthsPassive(obj)            
            value = obj.lengths(obj.cableIndicesPassive);
        end
        
        function value = get.forces(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.force;
            end
        end
        
        function set.forces(obj, f)
            CASPR_log.Assert(length(f) == obj.numCablesActive, 'Forces dimension inconsistent with the number of active cables');
            for i = 1:obj.numCablesActive
                obj.cables{obj.cableIndicesActive(i)}.force = f(i);
            end
        end
        
        function value = get.forcesActive(obj)            
            value = obj.forces(obj.cableIndicesActive);
        end
        
        function value = get.forcesActiveMin(obj)           
            value = zeros(obj.numCablesActive, 1);
            for i = 1:obj.numCablesActive
                value(i) = obj.cables{obj.cableIndicesActive(i)}.forceMin;
            end
        end
        
        function value = get.forcesActiveMax(obj)            
            value = zeros(obj.numCablesActive, 1);
            for i = 1:obj.numCablesActive
                value(i) = obj.cables{obj.cableIndicesActive(i)}.forceMax;
            end
        end
                
        function value = get.forcesPassive(obj)            
            value = obj.forces(obj.cableIndicesPassive);
        end
        
        function value = get.cableIndicesPassive(obj)            
            cableIndices = 1:obj.numCables;
            value = cableIndices(setdiff(1:length(cableIndices), obj.cableIndicesActive));
        end
        
        function value = get.numCablesPassive(obj)            
            value = obj.numCables - obj.numCablesActive;
        end        
        
        function value = get.V_active(obj)            
            value = obj.V(obj.cableIndicesActive, :);
        end
        
        function value = get.V_passive(obj)            
            value = obj.V(obj.cableIndicesPassive, :);
        end
        
        function value = get.numCables(obj)
            value = length(obj.cables);
        end
        
        function value = get.V_grad(obj)
            if obj.modelMode==ModelModeType.COMPILED || ~obj.modelOptions.isComputeHessian
                %CASPR_log.Warn('V_grad is not computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = obj.V_grad;
            end
        end
        
        function value = get.V_grad_active(obj)
            if obj.modelMode==ModelModeType.COMPILED || ~obj.modelOptions.isComputeHessian
                %CASPR_log.Warn('V_grad is not computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = obj.V_grad(obj.cableIndicesActive, :, :);
            end
        end
        
        function value = get.V_grad_passive(obj)
            if obj.modelMode==ModelModeType.COMPILED || ~obj.modelOptions.isComputeHessian
                %CASPR_log.Warn('V_grad is not computed under compiled mode or if hessian computation is off');
                value = [];
            else
                value = obj.V_grad(obj.cableIndicesPassive, :, :);
            end
        end
        
        function value = get.FORCES_ACTIVE_INVALID(obj)
            value = CableModelBase.INVALID_FORCE * ones(obj.numCablesActive, 1);
        end
        
        function value = get.isSymbolic(obj)
            value = (obj.modelMode == ModelModeType.SYMBOLIC);
        end
        
        
        % Compiling cable variables under COMPILED mode
        % - Symbolic Variables are compiled into .m files and saved to the
        % path
        function compile(obj, path, bodyModel, lib_name)   
            CASPR_log.Assert(obj.modelMode == ModelModeType.SYMBOLIC, 'Can only compile a symbolic model');
            
            CASPR_log.Info('Compiling Cable Variables...');
            CASPR_log.Info('- Compiling V...');
            matlabFunction(obj.V, 'File', strcat(path, '/', lib_name, '_compiled_V'), 'Vars', {bodyModel.q, bodyModel.q_dot, bodyModel.q_ddot, bodyModel.W_e});        
            CASPR_log.Info('- Compiling cable lengths...');           
            matlabFunction(obj.lengths, 'File', strcat(path, '/', lib_name, '_compiled_lengths'), 'Vars', {bodyModel.q, bodyModel.q_dot, bodyModel.q_ddot, bodyModel.W_e});     
            CASPR_log.Info('- Compiling r_OAs...');                             
            matlabFunction(obj.r_OAs, 'File', strcat(path, '/', lib_name, '_compiled_r_OAs'), 'Vars', {bodyModel.q, bodyModel.q_dot, bodyModel.q_ddot, bodyModel.W_e});                   
        end
    end
    
    methods (Access = private)
        function S_KA = generate_SKA_cross_rot(obj,k_a,k_b,k,bodyModel,r_OA)
            % Determine which is the smaller term
            if(k_a < k_b)
                sign_factor = 1;
                k_min = k_a;
                k_max = k_b;
            else
                sign_factor = -1;
                k_min = k_a;
                k_max = k_b;
            end
            R_k0 = bodyModel.bodies{k}.R_0k.';
            S_KA = MatrixOperations.Initialise([3,bodyModel.numDofs],obj.modelMode == ModelModeType.SYMBOLIC);
            if(k_min == 0 || bodyModel.bodiesPathGraph(k_min,k_max))
                if(k==k_min)
                    for i = k_min+1:k_max
                        if(bodyModel.bodiesPathGraph(i,k_max))
                            body_i = bodyModel.bodies{i};
                            R_0i = body_i.R_0k;
                            S_KA(:,bodyModel.qIndexInit(i):bodyModel.qIndexInit(i)+body_i.numDofs-1) = -sign_factor*R_k0*R_0i*MatrixOperations.SkewSymmetric(-body_i.r_OP + R_0i.'*r_OA)*bodyModel.S(6*i-2:6*i,bodyModel.qIndexInit(i):bodyModel.qIndexInit(i)+body_i.numDofs-1);
                        end
                    end
                elseif(k == k_max)
                    for i = k_min+1:k_max
                        if(bodyModel.bodiesPathGraph(i,k_max))
                            body_i = bodyModel.bodies{i};
                            R_0i = body_i.R_0k;
                            S_KA(:,bodyModel.qIndexInit(i):bodyModel.qIndexInit(i)+body_i.numDofs-1) = sign_factor*R_k0*R_0i*MatrixOperations.SkewSymmetric(body_i.r_OP - R_0i.'*r_OA)*bodyModel.S(6*i-2:6*i,bodyModel.qIndexInit(i):bodyModel.qIndexInit(i)+body_i.numDofs-1);
                        end
                    end
                end
            end
        end
        
        function S_KA = generate_SKA_trans(obj,k_a,k_b,k,bodyModel)   
            if(k_a < k_b)
                % THIS NEEDS TO BE INVESTIGATED TIME PENDING
                sign_factor = 1;
                k_min = k_a;
                k_max = k_b;
            else
                % Going backwards on the kinematic chain
                sign_factor = -1;
                k_min = k_b;
                k_max = k_a;
            end
            R_k0 = bodyModel.bodies{k}.R_0k.';
            S_KA = MatrixOperations.Initialise([3,bodyModel.numDofs],obj.modelMode == ModelModeType.SYMBOLIC);
            if(k_min == 0 || bodyModel.bodiesPathGraph(k_min,k_max))
                for i =k_min+1:k_max
                    if(bodyModel.bodiesPathGraph(i,k_max))
                        ip = bodyModel.bodies{i}.parentLinkId;
                        if(ip == 0)
                            R_0ip = eye(3);
                        else
                            body_ip = bodyModel.bodies{bodyModel.bodies{i}.parentLinkId};
                            R_0ip = body_ip.R_0k;
                        end
                        body_i = bodyModel.bodies{i};
                        S_KA(:,bodyModel.qIndexInit(i):bodyModel.qIndexInit(i)+body_i.numDofs-1) = sign_factor*R_k0*R_0ip*bodyModel.S(6*i-5:6*i-3,bodyModel.qIndexInit(i):bodyModel.qIndexInit(i)+body_i.numDofs-1);
                    end
                end
            end
        end
        
        function [k_A,k_B] = determine_anchor_links(obj,i,j,k,c_ijk)
            CASPR_log.Assert(abs(c_ijk)==1,'Anchor links can only be determined when c_ijk is unitary');
            if(c_ijk == 1)
                k_B = k;
                k_A = find(obj.getCRMTerm(i,j,1:obj.numLinks+1)==-1);
                k_A = k_A - 1; % To account for CRM indexing
            else
                k_A = k;
                k_B = find(obj.getCRMTerm(i,j,1:obj.numLinks+1)==1);
                k_B = k_B - 1; % To account for CRM indexing
            end
        end
        
        % This function updates V_grad
        function update_hessian(obj, bodyModel)
            is_symbolic = obj.modelMode == ModelModeType.SYMBOLIC;
            
            for i = 1:obj.numCables
                % Cables are already up to date
                cable = obj.cables{i};
                num_cable_segments = cable.numSegments;
                for k = 1:obj.numLinks
                    body_k = bodyModel.bodies{k};
                    V_ik_t_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],is_symbolic);
                    V_ik_r_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],is_symbolic);
                    for j = 1:num_cable_segments
                        c_ijk = obj.getCRMTerm(i,j,k+1);
                        if(c_ijk~=0)
                            segment = cable.segments{j};
                            % Initialisations
                            l_ij_grad = MatrixOperations.Initialise([3,bodyModel.numDofs],is_symbolic);
                            l_hat_ij_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],is_symbolic);
                            rot_l_hat_ij_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],is_symbolic);
                            [k_A,k_B] = obj.determine_anchor_links(i,j,k,c_ijk);
                            % Translational term is the same
                            % THIS CODE SHOULD BE MADE MORE CONSISTENT A
                            % LOT OF THIS SECTION IS UNNEEDED.
                            if(c_ijk == 1)
                                % This means that link k is the link of B_ij
                                % Compute the Translational term
                                % First deal with the translation derivative component
                                l_ij_grad = obj.generate_SKA_trans(min([k_A,k_B]),max([k_A,k_B]),k,bodyModel);
                                % Compute cross product term
                                l_ij_grad = l_ij_grad + obj.generate_SKA_cross_rot(min([k_A,k_B]),max([k_A,k_B]),k,bodyModel,segment.attachments{1}.r_OA);
                                l_ij = body_k.R_0k.'*segment.segmentVector;
                                l_ij_length = segment.length;
%                                 l_ij_hat = l_ij/l_ij_length;
                                l_hat_ij_grad(1,:,:) = ((1/l_ij_length)*eye(3) - (1/l_ij_length^3)*(l_ij*l_ij.'))*l_ij_grad;                                  
                                rot_l_hat_ij_grad(1,:,:) = MatrixOperations.SkewSymmetric(segment.attachments{2}.r_GA)*((1/segment.length)*eye(3) - (1/segment.length^3)*(l_ij*l_ij.'))*l_ij_grad;
                                V_ik_t_grad(1,:,:) = V_ik_t_grad(1,:,:) + l_hat_ij_grad;
                                V_ik_r_grad(1,:,:) = V_ik_r_grad(1,:,:) + rot_l_hat_ij_grad;
                            else
                                % This means that link k is the link of A_ij
                                % Compute the Translational term
                                % First deal with the translation derivative component
                                l_ij_grad = obj.generate_SKA_trans(min([k_A,k_B]),max([k_A,k_B]),k,bodyModel);
                                % Compute cross product term
                                l_ij_grad = l_ij_grad + obj.generate_SKA_cross_rot(min([k_A,k_B]),max([k_A,k_B]),k,bodyModel,segment.attachments{2}.r_OA);
                                l_hat_ij_grad(1,:,:) = ((1/segment.length)*eye(3) - (1/segment.length^3)*(segment.segmentVector*segment.segmentVector.'))*l_ij_grad;
                                rot_l_hat_ij_grad(1,:,:) = MatrixOperations.SkewSymmetric(segment.attachments{1}.r_GA)*((1/segment.length)*eye(3) - (1/segment.length^3)*(segment.segmentVector*segment.segmentVector.'))*l_ij_grad;
                                V_ik_t_grad = V_ik_t_grad - l_hat_ij_grad; % Subtraction accounts for the c_ijk multiplication
                                V_ik_r_grad(1,:,:) = V_ik_r_grad(1,:,:) - rot_l_hat_ij_grad;
                            end
                        end
                    end
                    % Translation term
                    obj.V_grad(i, 6*k-5:6*k-3,:) = V_ik_t_grad;
                    % Rotation term
                    obj.V_grad(i, 6*k-2:6*k,:) = V_ik_r_grad;
                end
            end   
        end
        
        % set update function
        function set_update_fns(obj) 
            CASPR_log.Assert(~isempty(obj.modelMode), 'The model mode should never empty');
            if obj.modelMode == ModelModeType.DEFAULT || obj.modelMode == ModelModeType.SYMBOLIC
                obj.update = @obj.defaultUpdate;
            elseif obj.modelMode == ModelModeType.COMPILED
                obj.update = @obj.compiledUpdate;
                obj.compiled_r_OAs_fn = str2func([obj.compiled_lib_name, '_compiled_r_OAs']);
                obj.compiled_lengths_fn = str2func([obj.compiled_lib_name, '_compiled_lengths']);
                obj.compiled_V_fn = str2func([obj.compiled_lib_name, '_compiled_V']);
            else
                CASPR_log.Error('Model mode does not exist');
            end
        end
    end


    methods (Static)
        function c = LoadXmlObj(cable_prop_xmlobj, bodiesModel, model_mode, model_options, compiled_lib_name)
            CASPR_log.Assert(strcmp(cable_prop_xmlobj.getNodeName, 'cable_set'), 'Root element should be <cable_set>');
            allCableItems = cable_prop_xmlobj.getChildNodes;
            num_cables = allCableItems.getLength;
            xml_cables = cell(1,num_cables);

            % Creates all of the links first
            for k = 1:num_cables
                % Java uses 0 indexing
                currentCableItem = allCableItems.item(k-1);

                type = char(currentCableItem.getNodeName);
                if (strcmp(type, 'cable_ideal'))
                    xml_cables{k} = CableModelIdeal.LoadXmlObj(currentCableItem, bodiesModel);
                elseif (strcmp(type, 'cable_linear_spring'))
                    xml_cables{k} = CableModelLinearSpring.LoadXmlObj(currentCableItem, bodiesModel);
                elseif (strcmp(type, 'cable_passive_linear_spring'))
                    xml_cables{k} = CableModelPassiveLinearSpring.LoadXmlObj(currentCableItem, bodiesModel);
                elseif (strcmp(type, 'cable_vsd_torsion_spring'))
                    xml_cables{k} = CableModelVSDTorsionSpring.LoadXmlObj(currentCableItem, bodiesModel);
                elseif (strcmp(type, 'cable_vsd_flexure_linear'))
                    xml_cables{k} = CableModelVSDFlexureLinear.LoadXmlObj(currentCableItem, bodiesModel);
                elseif (strcmp(type, 'cable_vsd_polynomial'))
                    xml_cables{k} = CableModelVSDPolynomial.LoadXmlObj(currentCableItem, bodiesModel);
                elseif (strcmp(type, 'muscle_hill_type'))
                    CASPR_log.Print('muscle_hill_type not implemented yet, please try again later.', CASPRLogLevel.ERROR);
                elseif (strcmp(type, 'pneumatic_artificial_muscle'))
                    CASPR_log.Print('pneumatic_artificial_muscle not implemented yet, please try again later.', CASPRLogLevel.ERROR);
                else
                    CASPR_log.Print(sprintf('Unknown cables type: %s', type),CASPRLogLevel.ERROR);
                end
            end

            % Create the actual object to return
            c = SystemModelCables(xml_cables, bodiesModel, model_mode, model_options, compiled_lib_name);
        end
    end
end
