% System kinematics of the cables for the system
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
%    Data structure that represents the kinematics for the cables of the
% system. It contains the kinematics of the set of cables, encapsulated
% within the CableKinematics object.
%   When the kinematics are updated, the V matrix (l_dot = V x_dot) also
%   gets updated and stored.
classdef SystemKinematicsCables < handle

    properties (SetAccess = private)
        % This matrix should probably be computed as needed (dependent
        % variable), but if it is a commonly used matrix (i.e. accessed
        % multiple times even if the system state does not change) then
        % storing it would be more efficient. However this means that
        % update must be performed through this class' update function and
        % not on the body's update directly. This makes sense since just
        % updating one cable without updating the others would cause
        % inconsistency anyway.
        V                       % Cable Jacobian l_dot = V x_dot
    end

    properties (SetAccess = protected)
        cables = {};            % cell array of CableKinematics object

        numCables = 0;
        numLinks = 0;
    end

    properties (Dependent)
        numSegmentsMax          % Maximum number of segments out of all of the cables
        lengths                 % vector of lengths for all of the cables
    end

    methods
        function ck = SystemKinematicsCables(cables, numLinks)
            ck.cables = cables;
            ck.numCables = length(cables);
            ck.numLinks = numLinks;
        end

        % Update the kinematics of the cables for the entire system using
        % the body kinematics. This update function should also be called
        % to update the entire system, rather than calling the update
        % function for each cable directly.
        function update(obj, bodyKinematics)
            assert(bodyKinematics.numLinks == obj.numLinks, 'Number of links between the cable and body kinematics must be consistent');

            % Set each cable's kinematics (absolute attachment locations
            % and segment vectors)
            for i = 1:obj.numCables
                obj.cables{i}.update(bodyKinematics);
            end

            % Determine V
            obj.V = MatrixOperations.Initialise(obj.numCables,6*obj.numLinks,isa(bodyKinematics.q,'sym'));
            for i = 1:obj.numCables
                for k = 1:obj.numLinks
                    % linkNum = k - 1
                    V_ixk_T = [0; 0; 0];
                    V_itk_T = [0; 0; 0];
                    for j = 1:obj.cables{i}.numSegments
                        V_ijk_T = obj.getCRMTerm(i,j,k+1)*bodyKinematics.bodies{k}.R_0k.'*obj.cables{i}.segments{j}.segmentVector/obj.cables{i}.segments{j}.length;
                        V_ixk_T = V_ixk_T + V_ijk_T;
                        V_itk_T = V_itk_T + cross(obj.cables{i}.segments{j}.r_GA{k+1}, V_ijk_T);
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
    end


    methods (Static)
        function c = LoadXmlObj(cable_prop_xmlobj, bodiesKin)
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
                    xml_cables{k} = CableKinematicsIdeal.LoadXmlObj(currentCableItem, bodiesKin);
                else
                    error('Unknown cables type: %s', type);
                end
            end

            % Create the actual object to return
            c = SystemKinematicsCables(xml_cables, bodiesKin.numLinks);
        end
    end
end