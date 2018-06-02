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
        
        % These vectors contain the indices of each active cable
        % corresponding to their index within the cables vector
        numCables = 0;              % The number of cables
        numCablesActive = 0;        % Number of active cables
        cableIndicesActive = [];    % Index of active cables in cables vector
        numLinks = 0;               % The number of links
        K                           % The matix of cable stiffnesses
        modelMode                   % A flag to indicate the need for symbolic computations
        filesCompiled               % whether files are compiled
        update
    end

    properties (Dependent)
        numSegmentsMax              % Maximum number of segments out of all of the cables
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
    end

    methods
        function ck = SystemModelCables(cables, numLinks,model_mode)
            ck.setModelMode(model_mode);
            ck.cables = cables;
            ck.numCables = length(cables);
            ck.numLinks = numLinks;            
        end

        % Update the kinematics of the cables for the entire system using
        % the body kinematics. 3 update functions are set for the 3 model modes.
        % This update function should also be called
        % to update the entire system, rather than calling the update
        % function for each cable directly.
        
        % Update function under DEFAULT mode
        % - update using numerical methods
        function defaultUpdate(obj, bodyModel)
            CASPR_log.Assert(bodyModel.numLinks == obj.numLinks, 'Number of links between the cable and body kinematics must be consistent');
            is_symbolic = obj.modelMode==ModelModeType.SYMBOLIC;
            
            % Set each cable's kinematics (absolute attachment locations
            % and segment vectors) and Determine V
            obj.V = MatrixOperations.Initialise([obj.numCables,6*obj.numLinks],is_symbolic);
            obj.K = MatrixOperations.Initialise([obj.numCables,obj.numCables],is_symbolic);
       
            obj.numCablesActive = 0;
            for i = 1:obj.numCables
                obj.cables{i}.update(bodyModel);
                cable = obj.cables{i};
                num_cable_segments = cable.numSegments;
                for k = 1:obj.numLinks
                    % linkNum = k - 1
                    V_ixk_T = [0; 0; 0];
                    V_itk_T = [0; 0; 0];
                    body = bodyModel.bodies{k};
                    R_0k = body.R_0k;

                    for j = 1:num_cable_segments
                        segment = cable.segments{j};
                        V_ijk_T = obj.getCRMTerm(i,j,k+1)*R_0k.'*segment.segmentVector/segment.length;
                        V_ixk_T = V_ixk_T + V_ijk_T;
                        if obj.getCRMTerm(i,j,k+1) == -1
                            V_itk_T = V_itk_T + cross(segment.attachments{1}.r_GA, V_ijk_T);
                        elseif obj.getCRMTerm(i,j,k+1) == 1
                            V_itk_T = V_itk_T + cross(segment.attachments{2}.r_GA, V_ijk_T);
                        end  
                    end
                    V_element = [V_ixk_T.' V_itk_T.'];
                    obj.V(i, 6*k-5:6*k) = V_element;
                end
                obj.K(i,i) = obj.cables{i}.K;

                if (cable.isActive)
                    obj.numCablesActive = obj.numCablesActive + 1;
                end   
            end   
                       
            ind_active = 1;
            obj.cableIndicesActive = zeros(obj.numCablesActive, 1);            
            obj.lengths = MatrixOperations.Initialise([obj.numCables,1],is_symbolic);
            for i = 1:obj.numCables
                % Active cable
                if (obj.cables{i}.isActive)
                    obj.cableIndicesActive(ind_active) = i;
                    ind_active = ind_active + 1;                    
                end               
                obj.lengths(i) = obj.cables{i}.length;                
            end               
            
            if(bodyModel.occupied.hessian)
                obj.updateHessian(bodyModel);
            end
        end
        
        % Update function for preparation of compilation
        % - Symbolic expressions are calculated for the variables
        % - Simplification takes a long time, but it makes compilation
        % easier and more chance to be successfully done
        function compiledPreparationUpdate(obj, bodyModel)
            CASPR_log.Assert(bodyModel.numLinks == obj.numLinks, 'Number of links between the cable and body kinematics must be consistent');
            is_symbolic = true;
            
            % Set each cable's kinematics (absolute attachment locations
            % and segment vectors) and Determine V
            obj.V = MatrixOperations.Initialise([obj.numCables,6*obj.numLinks],is_symbolic);
            obj.K = MatrixOperations.Initialise([obj.numCables,obj.numCables],is_symbolic);
           
            CASPR_log.Info('Calculating V...');
            obj.numCablesActive = 0;
            for i = 1:obj.numCables
                obj.cables{i}.update(bodyModel);
                cable = obj.cables{i};
                num_cable_segments = cable.numSegments;
                for k = 1:obj.numLinks
                    % linkNum = k - 1
                    V_ixk_T = [0; 0; 0];
                    V_itk_T = [0; 0; 0];
                    body = bodyModel.bodies{k};
                    R_0k = body.R_0k;

                    for j = 1:num_cable_segments
                        CASPR_log.Info(['- Cable: ',num2str(i),' Link: ',num2str(k),...
                                ' Segment: ',num2str(j)]);
                        segment = cable.segments{j}; 
                        V_ijk_T = obj.getCRMTerm(i,j,k+1)*R_0k.'*segment.segmentVector/segment.length;
                        V_ijk_T = simplify(V_ijk_T, 'Step', k*20);     
                        V_ixk_T = V_ixk_T + V_ijk_T;
                        if obj.getCRMTerm(i,j,k+1) == -1
                            V_itk_T = V_itk_T + cross(segment.attachments{1}.r_GA, V_ijk_T);
                        elseif obj.getCRMTerm(i,j,k+1) == 1
                            V_itk_T = V_itk_T + cross(segment.attachments{2}.r_GA, V_ijk_T);
                        end  
                    end
                    V_element = [V_ixk_T.' V_itk_T.'];                    
                    V_element = simplify(V_element, 'Step', k*20);                    
                    obj.V(i, 6*k-5:6*k) = V_element;
                end
                obj.K(i,i) = obj.cables{i}.K;

                if (cable.isActive)
                    obj.numCablesActive = obj.numCablesActive + 1;
                end                
                
            end 
            
            CASPR_log.Info('Simplifying V...');
            obj.V = simplify(obj.V, 'Step', 20);
                       
            ind_active = 1;
            obj.cableIndicesActive = zeros(obj.numCablesActive, 1);            
            obj.lengths = MatrixOperations.Initialise([obj.numCables,1],is_symbolic);
            for i = 1:obj.numCables
                % Active cable
                if (obj.cables{i}.isActive)
                    obj.cableIndicesActive(ind_active) = i;
                    ind_active = ind_active + 1;                    
                end               
                obj.lengths(i) = obj.cables{i}.length;                
            end               
            
            if(bodyModel.occupied.hessian)
                obj.updateHessian(bodyModel);
            end
        end
        
        % Update function under COMPILED mode
        % - Update using compiled files   
        function compiledUpdate(obj, bodyModel)            
            obj.V = compile_V(bodyModel.q, bodyModel.q_dot, bodyModel.q_ddot, bodyModel.W_e);           
            obj.lengths = compile_lengths(bodyModel.q, bodyModel.q_dot, bodyModel.q_ddot, bodyModel.W_e); 
       end
        
        % This function updates V_grad
        function updateHessian(obj,bodyModel)
            is_symbolic = obj.modelMode == ModelModeType.SYMBOLIC;
            
            obj.V_grad = MatrixOperations.Initialise([obj.numCables,6*obj.numLinks,bodyModel.numDofs],is_symbolic);
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
                            l_ij_grad = MatrixOperations.Initialise([3,bodyModel.numDofs],is_symbolic); %#ok<NASGU>
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
            if obj.modelMode==ModelModeType.COMPILED
                CASPR_log.Warn('You are not allowed to assess this variable under COMPILED mode');
                return;
            end
%             if(isempty(obj.V_grad))
%                 obj.updateHessian(obj.bodyModel);
%             end
            value = obj.V_grad;
        end
        
        function value = get.V_grad_active(obj)
            if obj.modelMode==ModelModeType.COMPILED
                CASPR_log.Warn('You are not allowed to assess this variable under COMPILED mode');
                return;
            end
            value = obj.V_grad(obj.cableIndicesActive, :, :);
        end
        
        function value = get.V_grad_passive(obj)
            if obj.modelMode==ModelModeType.COMPILED
                CASPR_log.Warn('You are not allowed to assess this variable under COMPILED mode');
                return;
            end
            value = obj.V_grad(obj.cableIndicesPassive, :, :);
        end
        
        function value = get.FORCES_ACTIVE_INVALID(obj)
            value = CableModelBase.INVALID_FORCE * ones(obj.numCablesActive, 1);
        end
        
        % Model Mode Related Functions %        
        
        % set update function
        function setupdate(obj, model_mode)  
           if model_mode==ModelModeType.COMPILED
               if obj.filesCompiled
                   % When compiled files are available
                   obj.update = @obj.compiledUpdate;
               else
                   % Preparation for compilations
                   obj.update = @obj.compiledPreparationUpdate;
               end   
           else
               % DEFAULT and SYMBOLIC 
               obj.update = @obj.defaultUpdate;
           end
        end
        % set model mode
        function setModelMode(obj, model_mode)
            obj.modelMode = model_mode;
            obj.setupdate(model_mode);
        end
        % set files compiled flag
        function setFilesCompiled(obj, value)
            obj.filesCompiled = value;
        end       
        
        % Compiling cable variables under COMPILED mode
        % - Symbolic Variables are compiled into .m files and saved to the
        % path
        function compile(obj, path, bodyModel)   
            CASPR_log.Info('- Compiling Cable Variables...');
            matlabFunction(obj.V, 'File', strcat(path, '/compile_V'), 'Vars', {bodyModel.q, bodyModel.q_dot, bodyModel.q_ddot, bodyModel.W_e});                   
            matlabFunction(obj.lengths, 'File', strcat(path, '/compile_lengths'), 'Vars', {bodyModel.q, bodyModel.q_dot, bodyModel.q_ddot, bodyModel.W_e});                   
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
    end


    methods (Static)
        function c = LoadXmlObj(cable_prop_xmlobj, bodiesModel,model_mode)
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
            c = SystemModelCables(xml_cables, bodiesModel.numLinks,model_mode);
        end
    end
end
