% System kinematics and dynamics of the cables for the system
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
        cables = {};                % Cell array of CableKinematics object
        numCables = 0;              % The number of cables
        numLinks = 0;               % The number of links
    end

    properties (Dependent)
        numSegmentsMax              % Maximum number of segments out of all of the cables
        lengths                     % vector of lengths for all of the cables
        forces                      % vector of forces from cables
        forcesMin                   % vector of min forces from cables
        forcesMax                   % vector of max forces from cables
        forcesInvalid               % vector of invalid forces
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
            assert(bodyModel.numLinks == obj.numLinks, 'Number of links between the cable and body kinematics must be consistent');
            % Set each cable's kinematics (absolute attachment locations
            % and segment vectors) and Determine V
            obj.V = MatrixOperations.Initialise([obj.numCables,6*obj.numLinks],isa(bodyModel.q,'sym'));
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
            end
        end
        

        % Returns the c_{ijk} element of the CRM
        % CRM is m x s x (p+1) matrix representing the cable-routing
        function c_ijk = getCRMTerm(obj, i, j, k)
            assert(i <= obj.numCables, 'Invalid cable number.');
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
        
        function value = get.forces(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.force;
            end
        end
        
        function set.forces(obj, f)
            assert(length(f) == obj.numCables, 'Forces dimension inconsistent with the number of cables');
            
            for i = 1:obj.numCables
                obj.cables{i}.force = f(i);
            end
        end
        
        function value = get.forcesMin(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.forceMin;
            end
        end
        
        function value = get.forcesMax(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.forceMax;
            end
        end
        
        function value = get.forcesInvalid(obj)
            value = zeros(obj.numCables, 1);
            for i = 1:obj.numCables
                value(i) = obj.cables{i}.forceInvalid;
            end
        end
        
        function value = get.numCables(obj)
            value = length(obj.cables);
        end
    end


    methods (Static)
        function c = LoadXmlObj(cable_prop_xmlobj, bodiesModel)
            assert(strcmp(cable_prop_xmlobj.getNodeName, 'cable_set'), 'Root element should be <cable_set>');
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
                    xml_cables{k} = CableModelLinearSpring.LoadXmlObj(currentCableItem);
                else
                    error('Unknown cables type: %s', type);
                end
            end

            % Create the actual object to return
            c = SystemModelCables(xml_cables, bodiesModel.numLinks);
        end
    end
end
