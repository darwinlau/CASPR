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
        % These vectors contain the indices of each active or passive cable
        % corresponding to their index within the cables vector
        cableIndicesActive = [];    % Index of active cables in cables vector
        cableIndicesPassive = [];   % Index of passive cables in cables vector
        numCables = 0;              % The number of cables
        numCablesPassive = 0;       % Number of passive cables
        numCablesActive = 0;        % Number of active cables
        numLinks = 0;               % The number of links
        K                           % The matix of cable stiffnesses
        is_symbolic                 % A flag to indicate the need for symbolic computations
    end

    properties (Dependent)
        numSegmentsMax              % Maximum number of segments out of all of the cables
        lengths                     % Vector of lengths for all of the cables
        lengthsActive               % Vector of lengths for active cables
        lengthsPassive              % Vector of lengths for passive cables
        forces                      % Vector of forces for all of the cables
        forcesActive                % Vector of forces for active cables
        forcesActiveMin             % Vector of min forces for active cables
        forcesActiveMax             % Vector of max forces for passive cables
        forcesPassive               % Vector of forces for passive cables
        V_active
        V_passive
        V_grad_active
        V_grad_passive
        FORCES_ACTIVE_INVALID       % Constant of vector of invalid forces for active cables
    end

    methods
        function ck = SystemModelCables(cables, numLinks)
            ck.cables = cables;
            ck.numCables = length(cables);
            ck.numLinks = numLinks;
        end

        % Update the kinematics of the cables for the entire system using
        % the body kinematics. This update function should also be called
        % to update the entire system, rather than calling the update
        % function for each cable directly.
        function update(obj, bodyModel)
            CASPR_log.Assert(bodyModel.numLinks == obj.numLinks, 'Number of links between the cable and body kinematics must be consistent');
            obj.is_symbolic = isa(bodyModel.q,'sym');
            % Set each cable's kinematics (absolute attachment locations
            % and segment vectors) and Determine V
            obj.V = MatrixOperations.Initialise([obj.numCables,6*obj.numLinks],obj.is_symbolic);
            obj.K = MatrixOperations.Initialise([obj.numCables,obj.numCables],obj.is_symbolic);
            
            obj.numCablesActive = 0;
            obj.numCablesPassive = 0;
            for i = 1:obj.numCables
                obj.cables{i}.update(bodyModel);
                cable = obj.cables{i};
                num_cable_segments = cable.numSegments;
                for k = 1:obj.numLinks
                    % linkNum = k - 1
                    V_ixk_T = [0; 0; 0];
                    V_itk_T = [0; 0; 0];
                    body = bodyModel.bodies{k};
                    for j = 1:num_cable_segments
                        segment = cable.segments{j};
                        V_ijk_T = obj.getCRMTerm(i,j,k+1)*body.R_0k.'*segment.segmentVector/segment.length;
                        V_ixk_T = V_ixk_T + V_ijk_T;
                        V_itk_T = V_itk_T + cross(segment.r_GA{k+1}, V_ijk_T);
                    end
                    obj.V(i, 6*k-5:6*k) = [V_ixk_T.' V_itk_T.'];
                end
                obj.K(i,i) = obj.cables{i}.K;
                
                if (cable.isActive)
                    obj.numCablesActive = obj.numCablesActive + 1;
                else
                    obj.numCablesPassive = obj.numCablesPassive + 1;
                end
            end
            
%             if(obj.is_symbolic)
%                 for i = size(obj.V,1)
%                     for j =1:size(obj.V,2)
%                         obj.V(i,j) = expand(obj.V(i,j),'ArithmeticOnly',true);
%                         obj.V(i,j) = simplify(obj.V(i,j),10);
%                     end
%                 end
%             end
            
            ind_active = 1;
            ind_passive = 1;
            obj.cableIndicesActive = zeros(obj.numCablesActive, 1);
            obj.cableIndicesPassive = zeros(obj.numCablesPassive, 1);
            for i = 1:obj.numCables
                % Active cable
                if (obj.cables{i}.isActive)
                    obj.cableIndicesActive(ind_active) = i;
                    ind_active = ind_active + 1;
                % Passive cable
                else
                    obj.cableIndicesPassive(ind_passive) = i;
                    ind_passive = ind_passive + 1;
                end
            end
            
            if(bodyModel.occupied.hessian)
                obj.updateHessian(bodyModel);
            end
        end
        
        % This function updates V_grad
        function updateHessian(obj,bodyModel)
            obj.V_grad = MatrixOperations.Initialise([obj.numCables,6*obj.numLinks,bodyModel.numDofs],obj.is_symbolic);
            for i = 1:obj.numCables
                % Cables are already up to date
                cable = obj.cables{i};
                num_cable_segments = cable.numSegments;
                for k = 1:obj.numLinks
                    body_k = bodyModel.bodies{k};
                    V_ik_t_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],obj.is_symbolic);
                    V_ik_r_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],obj.is_symbolic);
                    for j = 1:num_cable_segments
                        c_ijk = obj.getCRMTerm(i,j,k+1);
                        if(c_ijk)
                            segment = cable.segments{j};
                            % Initialisations
                            l_ij_grad = MatrixOperations.Initialise([3,bodyModel.numDofs],obj.is_symbolic); %#ok<NASGU>
                            l_hat_ij_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],obj.is_symbolic);
                            rot_l_hat_ij_grad = MatrixOperations.Initialise([1,3,bodyModel.numDofs],obj.is_symbolic);
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
                                l_ij_grad = l_ij_grad + obj.generate_SKA_cross_rot(min([k_A,k_B]),max([k_A,k_B]),k,bodyModel,cable.segments{j}.r_OA{k_A+1});
                                l_ij = body_k.R_0k.'*segment.segmentVector;
                                l_hat_ij_grad(1,:,:) = ((1/segment.length)*eye(3) - (1/segment.length^3)*(l_ij*l_ij.'))*l_ij_grad;
                                rot_l_hat_ij_grad(1,:,:) = MatrixOperations.SkewSymmetric(segment.r_GA{k+1})*((1/segment.length)*eye(3) - (1/segment.length^3)*(l_ij*l_ij.'))*l_ij_grad;
                                V_ik_t_grad(1,:,:) = V_ik_t_grad(1,:,:) + l_hat_ij_grad;
                                V_ik_r_grad(1,:,:) = V_ik_r_grad(1,:,:) + rot_l_hat_ij_grad;
                            else
                                % This means that link k is the link of A_ij
                                % Compute the Translational term
                                % First deal with the translation derivative component
                                l_ij_grad = obj.generate_SKA_trans(min([k_A,k_B]),max([k_A,k_B]),k,bodyModel);
                                % Compute cross product term
                                l_ij_grad = l_ij_grad + obj.generate_SKA_cross_rot(min([k_A,k_B]),max([k_A,k_B]),k,bodyModel,cable.segments{j}.r_OA{k_B+1});
                                l_hat_ij_grad(1,:,:) = ((1/segment.length)*eye(3) - (1/segment.length^3)*(segment.segmentVector*segment.segmentVector.'))*l_ij_grad;
                                rot_l_hat_ij_grad(1,:,:) = MatrixOperations.SkewSymmetric(segment.r_GA{k+1})*((1/segment.length)*eye(3) - (1/segment.length^3)*(segment.segmentVector*segment.segmentVector.'))*l_ij_grad;
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
        
        

        %         function C = getCRM(obj)
        %             C = zeros(obj.numCables, obj.numSegmentsMax, obj.numLinks+1);
        %             for i = 1:obj.numCables
        %                 C(i,:,:)
        %             end
        %             obj.cables{:}.numSegments
        %         end

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
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.length;
            end
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
%             if(isempty(obj.V_grad))
%                 obj.updateHessian(obj.bodyModel);
%             end
            value = obj.V_grad;
        end
        
        function value = get.V_grad_active(obj)
            value = obj.V_grad(obj.cableIndicesActive, :, :);
        end
        
        function value = get.V_grad_passive(obj)
            value = obj.V_grad(obj.cableIndicesPassive, :, :);
        end
        
        function value = get.FORCES_ACTIVE_INVALID(obj)
            value = CableModelBase.INVALID_FORCE * ones(obj.numCablesActive, 1);
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
            S_KA = MatrixOperations.Initialise([3,bodyModel.numDofs],obj.is_symbolic);
            if(k_min == 0 || bodyModel.bodiesPathGraph(k_min,k_max))
                if(k==k_min)
                    for i = k_min+1:k_max
                        if(bodyModel.bodiesPathGraph(i,k_max))
                            body_i = bodyModel.bodies{i};
                            R_0i = body_i.R_0k;
                            S_KA(:,bodyModel.index_k(i):bodyModel.index_k(i)+body_i.numDofs-1) = -sign_factor*R_k0*R_0i*MatrixOperations.SkewSymmetric(-body_i.r_OP + R_0i.'*r_OA)*bodyModel.S(6*i-2:6*i,bodyModel.index_k(i):bodyModel.index_k(i)+body_i.numDofs-1);
                        end
                    end
                elseif(k == k_max)
                    for i = k_min+1:k_max
                        if(bodyModel.bodiesPathGraph(i,k_max))
                            body_i = bodyModel.bodies{i};
                            R_0i = body_i.R_0k;
                            S_KA(:,bodyModel.index_k(i):bodyModel.index_k(i)+body_i.numDofs-1) = sign_factor*R_k0*R_0i*MatrixOperations.SkewSymmetric(body_i.r_OP - R_0i.'*r_OA)*bodyModel.S(6*i-2:6*i,bodyModel.index_k(i):bodyModel.index_k(i)+body_i.numDofs-1);
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
            S_KA = MatrixOperations.Initialise([3,bodyModel.numDofs],obj.is_symbolic);
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
                        S_KA(:,bodyModel.index_k(i):bodyModel.index_k(i)+body_i.numDofs-1) = sign_factor*R_k0*R_0ip*bodyModel.S(6*i-5:6*i-3,bodyModel.index_k(i):bodyModel.index_k(i)+body_i.numDofs-1);
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
                k_B = find(obj.getCRMTerm(i,j,:)==1);
                k_B = k_B - 1; % To account for CRM indexing
            end
        end 
    end


    methods (Static)
        function c = LoadXmlObj(cable_prop_xmlobj, bodiesModel)
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
                elseif (strcmp(type, 'muscle_hill_type'))
                    CASPR_log.Print('muscle_hill_type not implemented yet, please try again later.',CASPRLogLevel.ERROR);
                elseif (strcmp(type, 'pneumatic_artificial_muscle'))
                    CASPR_log.Print('pneumatic_artificial_muscle not implemented yet, please try again later.',CASPRLogLevel.ERROR);
                else
                    CASPR_log.Print(sprintf('Unknown cables type: %s', type),CASPRLogLevel.ERROR);
                end
            end

            % Create the actual object to return
            c = SystemModelCables(xml_cables, bodiesModel.numLinks);
        end
    end
end
