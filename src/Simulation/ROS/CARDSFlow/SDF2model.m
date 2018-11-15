% Convert SDF files to model and cables files
%
% Author        : Dominic Chan
% Created       : 2018
% Description    :
%    This class is a class for the convertion from SDF files to the model
%    files used in CASPR
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       WARNING                       %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The current version only supports kinematic chains with only revolute and
% prismatic joints. (NO spatial joints/free joints) Fixed joints are
% ignored.
%
% Please make sure every link except the base, must only have one parent
% (no. of links = no. of joints + 1)

classdef SDF2model < handle  
    
    properties 
       model_folder         % Folder for saving the model and cables files
       sdf_filepath         % Filename of the sdf
       xml_filepath         % Filename of the xml
       
       f_min                % Min cable force bound
       f_max                % Max cable force bound
       
       model_name           % Name of the model
       links                % Struct saving the links info
       joints               % Struct saving the joints info
       cables               % Struct saving the cables info     
       
       isRotate             % Flag for applying that sTrAnGe rotation of Fusion
    end   
    
    methods
        % Constructor
        % Enter the path where you want the model xmls to be created
        function obj = SDF2model(model_folder)
            obj.model_folder = model_folder;             
        end       
        
        %%%%%%%%%%%%%%%
        % Convert SDF %
        %%%%%%%%%%%%%%%
        % 1. sdf_filepath   : full path of the sdf file (Links & joints)
        % 2. xml_filepath   : full path of the xml file (Cables)
        % 3. f_min, f_max   : min and max cable forces
        % 4. isRotate       : flag for whether the strange rotation matrix 
        %                     is needed for Fusion's reference frame         
        function convert(obj, sdf_filepath, xml_filepath, f_min, f_max, isRotate)
            obj.sdf_filepath = sdf_filepath;
            obj.xml_filepath = xml_filepath;
            obj.f_min = f_min;
            obj.f_max = f_max;
            
            % Whether the rotation matrix is needed or not
            if nargin > 5               
                obj.isRotate = isRotate;            
            else               
                obj.isRotate = false;
            end  
            
            % Parse SDF
            obj.parseSDF();
            % Parse XML
            obj.parseXML();
            % Create bodies.xml
            obj.createBodiesXml();            
            % Create cables.xml
            obj.createCablesXml();
            % Create traj.xml
            obj.createTrajXml();
            
            % Finish message
            obj.finishMessage();
        end
        
        % set model folder
        function setModelFolder(obj, model_folder)
            obj.model_folder = model_folder;             
        end                         
    end    
    
    methods (Access = private)
        % Parser of the sdf
        function parseSDF(obj)
            delimiter = '\t';
            formatSpec = '%q%q%q%q%q%q%q%[^\n\r]';
            % Open the file as text
            fileID = fopen(obj.sdf_filepath,'r');

            % Read columns of data according to the format            
            dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);

            % Close the text file.
            fclose(fileID);

            % save the data as a string array
            model_str = [dataArray{1:end-1}];
            
            % Remove all comments
            model_str = obj.removeComments(model_str);
            
            % Find model name            
            obj.model_name = obj.findAttribute(model_str, 'model', 'name');             
   
            % Identify links
            obj.identifyLinks(model_str);          
        end
        
        % Parser of the xml
        function parseXML(obj)
            delimiter = '\t';
            formatSpec = '%q%[^\n\r]';
            formatSpec = '%q';
            % Open the file as text
            fileID = fopen(obj.xml_filepath,'r');

            % Read columns of data according to the format            
            dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);

            % Close the text file.
            fclose(fileID);

            % save the data as a string array
            model_str = [dataArray{1}];
            
            % Remove all comments
            model_str = obj.removeComments(model_str);
            
            % Identify cables
            obj.identifyCables(model_str);
        end
        
        % Remove comments
        function [str] = removeComments(~, str)
            % Delete rows with the start of comments
            strfind_idx = contains(str, '<!--');            
            for i = size(str,1):-1:1
                for j = 1:size(str,2)
                    if strfind_idx(i,j) 
                        str(i,:) = [];
                        break;
                    end
                end
            end
            % Delete rows with the end of comments
            strfind_idx = contains(str, '-->');            
            for i = size(str,1):-1:1
                for j = 1:size(str,2)
                    if strfind_idx(i,j) 
                        str(i,:) = [];
                        break;
                    end
                end
            end
        end
        
        % Return array of starting position and ending position of element found
        function [start_idx, end_idx] = findElement(~, str, element)           
            start_idx = find(contains(str, ['<', element]));         
            end_idx = find(contains(str, ['</', element])); 
        end  
        
        % Return an array of attribute found
        function [attributes, idx] = findAttribute(~, str, element, attribute)
            % Find the element str
            idx = find(contains(str, ['<', element]));         
            element_str = str(idx);
                        
            % Formulate the return attributes
            attributes = [];
            for i =1:size(element_str,1)     
                % Find the attribute str
                split_array = regexp(element_str(i), ' ', 'split');                
                attribute_str = split_array(contains(split_array, attribute));
                
                if ~isempty(attribute_str)
                    % Find the attribute
                    attribute_start_idx = strfind(attribute_str, '=');
                    attribute_char_array = char(attribute_str(1,:));
                    % See if '>' exist
                    if contains(attribute_str, '>')
                        arrow_idx = strfind(attribute_str, '>');
                        attributes = [attributes; convertCharsToStrings(attribute_char_array(1, ...
                            attribute_start_idx+2:arrow_idx-2))];
                    else
                        attributes = [attributes; convertCharsToStrings(attribute_char_array(1, ...
                            attribute_start_idx+2:end-1))];
                    end  
                else
                    idx(i) = [];
                end       
            end            
        end  
        
        % Return the text of the element
        function [text] = findText(~, str, element)
            % Find the element str
            element_str = str(contains(str, ['<', element]));
                        
            % Formulate the return attributes
            text = [];
            for i = 1:size(element_str,1)   
                current_element_str = element_str(i);
                % Find the attribute
                text_start_idx = strfind(current_element_str, '>');
                text_end_idx = strfind(current_element_str, '</');                
                text_char_array = char(current_element_str);
                
                text = [text; convertCharsToStrings(text_char_array(text_start_idx(1)+1:text_end_idx(1)-1))]; 
            end            
        end  
                
        % Identify links
        function identifyLinks(obj, str)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            % Basic info - Joints
            % - Parents & Children
            parents = obj.findText(str, 'parent');            
            children = obj.findText(str, 'child');    
            
            obj.joints.Names = unique(obj.findAttribute(str, 'joint', 'name'), 'stable');   
            obj.joints.Types = obj.findAttribute(str,'joint','type');
            original_n_joints = length(obj.joints.Names);
            % Discard all fixed joints            
            fixed_joint_logic = ~or(strcmp(obj.joints.Types,'revolute'), ...
                strcmp(obj.joints.Types,'prismatic'));
            obj.joints.Names(fixed_joint_logic) = [];
            obj.joints.Types(fixed_joint_logic) = [];            
            obj.joints.n_joints = length(obj.joints.Names);   
            obj.joints.n_fixed_joints = original_n_joints - obj.joints.n_joints;
            
            % Record fixed links and discard later
            obj.links.fixed_links = children(fixed_joint_logic);
            obj.links.n_links = obj.joints.n_joints+1;
            parents(fixed_joint_logic) = [];
            children(fixed_joint_logic) = [];    
            obj.joints.parents = parents;
            obj.joints.children = children;            
            
            % Joint infos
            % - Filter out valid joint poses
            pose_vec = obj.findText(str, 'pose');            
            joint_pose_vec = pose_vec(obj.links.n_links+...
                obj.joints.n_fixed_joints+1 : obj.links.n_links+obj.joints.n_joints+2*obj.joints.n_fixed_joints);
            joint_pose_vec(fixed_joint_logic) = [];
            joint_pos = cell(1, obj.joints.n_joints);            
            % - Other joint infos
            axes = cell(1, obj.joints.n_joints);
            axes_vec = obj.findText(str, 'xyz');            
            q_min = cell(1, obj.joints.n_joints);
            q_min_vec = obj.findText(str, 'lower');
            q_max = cell(1, obj.joints.n_joints);
            q_max_vec = obj.findText(str, 'upper');
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
            % Basic Info - Links
            % - COM info
            com_vec = pose_vec(1:obj.links.n_links+obj.joints.n_fixed_joints);
            obj.links.Names = unique(obj.findAttribute(str,'link','name'),'stable');
            % Filter out fixed links
            for i = 1:length(obj.links.fixed_links)
                fixed_link_index = find(obj.links.Names==obj.links.fixed_links(i));
                obj.links.Names(fixed_link_index) = [];
                com_vec(fixed_link_index) = [];
            end            
            obj.links.Index = -1*ones(obj.links.n_links,1);
            obj.links.parentIndex = zeros(obj.links.n_links,1);  
            
            % Mass and Inertia
            mass = cell(1, obj.links.n_links);
            mass_vec = obj.findText(str', 'mass');
            inertia = cell(1, obj.links.n_links);
            ixx_vec = obj.findText(str, 'ixx');
            iyy_vec = obj.findText(str, 'iyy');
            izz_vec = obj.findText(str, 'izz');
            ixy_vec = obj.findText(str, 'ixy');
            ixz_vec = obj.findText(str, 'ixz');
            iyz_vec = obj.findText(str, 'iyz');
            
            % Locations
            com_pos = cell(1, obj.links.n_links);
            end_pos = cell(1, obj.links.n_links);
            parent_pos = cell(1, obj.links.n_links); 
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
            % Define link num by kinematic tree
            isFirst = 1;
            link_count = 0;
            while ~isempty(parents) 
                if isFirst
                    % Fixed joint
                    roots = setdiff(parents, children);
                    isFirst = 0;
                else
                    % Find the set difference, which means the root of tree
                    roots = setdiff(parents, children);                    
                end
                root = roots(1);
                % Define link index
                obj.links.Index(obj.links.Names == root) = link_count;
                
                % Update parent indices
                root_logic = parents == root;                
                children_names = children(root_logic);                   
                for j = 1:length(children_names)          
                    children_link_index = find(obj.links.Names == children_names(j));
                    obj.links.parentIndex(children_link_index) = link_count;
                end
                                
                % Update array
                parents(root_logic) = [];
                children(root_logic) = [];
                link_count = link_count + 1;              
            end
            
            % Fill up smallest children
            children_count = max(obj.links.Index) + 1;
            for i = 1:length(obj.links.Index)
                if obj.links.Index(i) == -1
                    obj.links.Index(i) = children_count;
                    children_count = children_count + 1;
                end                
            end            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                        
            % Looping through links by link_num          
            link_num = 1;
            % 1. Loop through the links to get info (joint)
            for i=1:obj.links.n_links - 1              
                % Find link corresponding to link_num
                link_index = find(obj.links.Index == link_num);                
                current_link_name = obj.links.Names(link_index);
                current_parent_name = obj.links.Names(obj.links.Index == obj.links.parentIndex(link_index));                
                % Find corresponding joint 
                parent_logic = obj.joints.parents == current_parent_name;
                children_logic = obj.joints.children == current_link_name;
                joint_index = find(and(parent_logic, children_logic) == 1);
                    
                % Find corresponding joint info                
                child_com = XmlOperations.StringToVector(com_vec(link_index));
                joint_offset = XmlOperations.StringToVector(joint_pose_vec(joint_index));
                joint_pos{joint_index} = child_com + joint_offset;
                axes{joint_index} = XmlOperations.StringToVector(axes_vec(joint_index));
                q_min{joint_index} = XmlOperations.StringToVector(q_min_vec(joint_index));
                q_max{joint_index} = XmlOperations.StringToVector(q_max_vec(joint_index));
                                
                % Find corresponding pose info
                mass{link_index} = XmlOperations.StringToVector(mass_vec(link_index));
                inertia{link_index} = [XmlOperations.StringToVector(ixx_vec(link_index));
                    XmlOperations.StringToVector(iyy_vec(link_index));
                    XmlOperations.StringToVector(izz_vec(link_index));
                    XmlOperations.StringToVector(ixy_vec(link_index));
                    XmlOperations.StringToVector(ixz_vec(link_index));
                    XmlOperations.StringToVector(iyz_vec(link_index))];                
                
                link_num = link_num + 1;
            end    
            obj.joints.Position = joint_pos;
            obj.joints.Axes = axes;
            obj.joints.q_min = q_min;
            obj.joints.q_max = q_max;
            
            % 2. Loop through the links based on joint positions
            link_num = 1;
            % Loop through the links to get info
            for i=1:obj.links.n_links - 1                
                % Find link corresponding to link_num
                link_index = find(obj.links.Index == link_num);                
                current_link_name = obj.links.Names(link_index);
                current_parent_name = obj.links.Names(obj.links.Index == obj.links.parentIndex(link_index));                
                % Find corresponding joint 
                parent_logic = obj.joints.parents == current_parent_name;
                children_logic = obj.joints.children == current_link_name;
                joint_index = find(and(parent_logic, children_logic) == 1);
                
                % Get the absolute COM position 
                abs_com = XmlOperations.StringToVector(com_vec(link_index));                 
                com_pos{link_index} = abs_com - obj.joints.Position{joint_index};
                children_link_index = find(obj.joints.parents==current_link_name);
                
                % Select the joint position of one of the children as the
                % end_position
                % If no children, then double the COM position to fake an
                % ending point
                if ~isempty(children_link_index)     
                    children_joint_index = find(obj.joints.children == obj.joints.children(children_link_index(1)),1);
                    end_pos{link_index} = obj.joints.Position{children_joint_index} - obj.joints.Position{joint_index};
                else
                    end_pos{link_index} = 2*com_pos{link_index};
                end
                
                link_num = link_num + 1;
            end
            
            % 3. Loop through the links to define the frame position with
            % respect to the parent link
            base_i = find(cellfun('isempty', end_pos));
            for i = 1:obj.links.n_links                 
                parent_num = obj.links.parentIndex(i);
                if i ~= base_i % Not the base
                    current_link_name = obj.links.Names(i);
                    current_parent_name = obj.links.Names(obj.links.Index == obj.links.parentIndex(i));                
                    % Find corresponding joint 
                    parent_logic = obj.joints.parents == current_parent_name;
                    children_logic = obj.joints.children == current_link_name;
                    joint_index = find(and(parent_logic, children_logic) == 1);
                    if parent_num == 0 % Parent is base                        
                        base_com = XmlOperations.StringToVector(com_vec(base_i));
                        parent_pos{i} = obj.joints.Position{joint_index} - base_com;
                    else % Otherwise
                        % Find parent's joint position
                        grand_parent_name = obj.joints.parents(obj.joints.children==current_parent_name);
                        grand_parent_logic = obj.joints.parents == grand_parent_name;
                        parent_child_logic = obj.joints.children == current_parent_name;
                        parent_joint_index = find(and(parent_child_logic, grand_parent_logic) == 1);
                        parent_pos{i} = obj.joints.Position{joint_index} - obj.joints.Position{parent_joint_index};
                    end      
                end                
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
            % End-effector Position
            % Check if ee pos defined in the sdf
            ee_relative_pos = obj.findText(str, 'endEffector');
            if ~isempty(ee_relative_pos)
                ee_relative_pos = XmlOperations.StringToVector(ee_relative_pos);
                % Find ee link com
                ee_link_name = obj.findAttribute(str, 'endEffector', 'link');
                ee_com = com_pos{contains(obj.links.Names, ee_link_name)};
                ee_com = ee_com(1:size(ee_relative_pos,1));
                obj.links.ee = ee_com + ee_relative_pos;
            else
                % Default define ee to smallest child
                smallest_child_logic = obj.links.Index == max(obj.links.Index);
                ee_pos = end_pos{smallest_child_logic};
                obj.links.ee = ee_pos(1:3);
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
            % Save to struct
            obj.links.Mass = mass;
            obj.links.Inertia = inertia;
            obj.links.com = com_pos;
            obj.links.end = end_pos;
            obj.links.parent = parent_pos;
        end        
        
        % Identify cables
        function identifyCables(obj, str)
            % Number of cables
            obj.cables.Names = obj.findAttribute(str, 'myoMuscle', 'name');
            obj.cables.n_cables = length(obj.cables.Names);
            obj.cables.n_invalid_cables = 0;
            
            % Find the starting and ending positions of the myomuscle
            % element
            [start_cable_idx, end_cable_idx] = obj.findElement(str, 'myoMuscle');            
            
            cable_array = cell(1,0);            
            % Loop through each cable
            for i=1:obj.cables.n_cables
                starting_row = mod(start_cable_idx(i), size(str,1));
                ending_row = mod(end_cable_idx(i), size(str,1));
                % Identify how many links attached
                cable_str = str(starting_row:ending_row,:);
                [start_link_idx, end_link_idx] = obj.findElement(cable_str, 'link');
                
                if length(start_link_idx) <= 1
                    obj.cables.n_invalid_cables = obj.cables.n_invalid_cables + 1;
                    continue;
                end
                
                current_cable = cell(1,0);
                % Loop through each link
                for j=1:length(start_link_idx)                    
                    starting_link_row = mod(start_link_idx(j), size(cable_str,1));
                    ending_link_row = mod(end_link_idx(j), size(cable_str,1));
                    link_str = cable_str(starting_link_row:ending_link_row,:);
                    % Link num from name
                    link_name = obj.findAttribute(link_str, 'link', 'name'); 
                    link_index = find(obj.links.Names==link_name);
                    link_num = obj.links.Index(link_index);
                    
                    % Via points
                    via_points_for_link = obj.findText(link_str, 'viaPoint');
                    % Add via points to cable via points
                    for k = 1:length(via_points_for_link)
                        current_via_point.link_num = link_num;
                        current_via_point.via_point = XmlOperations.StringToVector(via_points_for_link(k));
                        current_cable{end+1} = current_via_point;
                    end
                end
                % Add current cable to cables
                cable_array{end+1} = current_cable;
            end
            % Assign field to struct 
            obj.cables.cable_array = cable_array;
        end        
        
        % Create bodies xml
        function createBodiesXml(obj)            
            docNode = com.mathworks.xml.XMLUtils.createDocument('bodies_system');
            domImpl = docNode.getImplementation();
            doctype = domImpl.createDocumentType('bodies_system', '', '../../../templates/bodies.dtd');
            docNode.appendChild(doctype);
            % root node
            bodies_system_node = docNode.getDocumentElement;
            % Links
            links_node = docNode.createElement('links');
            links_node.setAttribute('display_range','-1.0 1.0 -1.0 1.0 -1.0 1.0');
            links_node.setAttribute('view_angle', '45 32');
            bodies_system_node.appendChild(links_node);
            
            % Rotation matrix for fusion STRANGE coordinate system
            if obj.isRotate
                R = [1,0,0;0,0,-1;0,1,0];
            else
                R = eye(3);
            end     
            
            link_num = 1;
            for i=1:obj.links.n_links - 1
                % Find link corresponding to link_num
                link_index = find(obj.links.Index == link_num);
                % Find joint index
                current_link_name = obj.links.Names(link_index);
                current_parent_name = obj.links.Names(obj.links.Index == obj.links.parentIndex(link_index));                
                % Find corresponding joint                 
                parent_logic = obj.joints.parents == current_parent_name;
                children_logic = obj.joints.children == current_link_name;
                joint_index = find(and(parent_logic, children_logic) == 1);
                                
                % Basic info
                current_link_node = docNode.createElement('link_rigid');
                current_link_node.setAttribute('num', num2str(link_num));
                current_link_node.setAttribute('name', obj.links.Names(link_index));
                links_node.appendChild(current_link_node);
                
                % Joint
                current_joint_node = docNode.createElement('joint');
                if strcmp(obj.joints.Types{joint_index}, 'revolute')
                    current_joint_node.setAttribute('type', 'R_AXIS');
                elseif strcmp(obj.joints.Types{joint_index},'prismatic')
                    current_joint_node.setAttribute('type', 'P_AXIS');
                end                
                current_joint_node.setAttribute('q_initial', '0.0');
                q_min_vec = cell2mat(obj.joints.q_min(joint_index));
                current_joint_node.setAttribute('q_min', obj.vec2chr(q_min_vec));
                q_max_vec = cell2mat(obj.joints.q_max(joint_index));
                current_joint_node.setAttribute('q_max', obj.vec2chr(q_max_vec));
                axis_vec = cell2mat(obj.joints.Axes(joint_index)); 
                current_joint_node.setAttribute('axis', obj.vec2chr(R*axis_vec));
                current_link_node.appendChild(current_joint_node);
                
                % Physical
                current_physical_node = docNode.createElement('physical');
                current_link_node.appendChild(current_physical_node);
                
                % Mass
                current_mass_node = docNode.createElement('mass');                
                mass_vec = cell2mat(obj.links.Mass(link_index));
                current_mass_node.appendChild(docNode.createTextNode(obj.vec2chr(mass_vec)));
                current_physical_node.appendChild(current_mass_node);
                
                % COM
                current_com_node = docNode.createElement('com_location');
                com_vec = cell2mat(obj.links.com(link_index));
                current_com_node.appendChild(docNode.createTextNode(obj.vec2chr(R*com_vec(1:3))));
                current_physical_node.appendChild(current_com_node);
                
                % END
                current_end_node = docNode.createElement('end_location');
                end_vec = cell2mat(obj.links.end(link_index));
                current_end_node.appendChild(docNode.createTextNode(obj.vec2chr(R*end_vec(1:3))));
                current_physical_node.appendChild(current_end_node);
                
                % Inertia
                current_inertia_node = docNode.createElement('inertia');
                current_inertia_node.setAttribute('ref', 'com');
                current_physical_node.appendChild(current_inertia_node);
                
                inertia_vec = cell2mat(obj.links.Inertia(link_index));                
                ixx_node = docNode.createElement('Ixx');                
                ixx_node.appendChild(docNode.createTextNode(obj.vec2chr(inertia_vec(1))));
                current_inertia_node.appendChild(ixx_node);
                iyy_node = docNode.createElement('Iyy');                
                iyy_node.appendChild(docNode.createTextNode(obj.vec2chr(inertia_vec(2))));
                current_inertia_node.appendChild(iyy_node);
                izz_node = docNode.createElement('Izz');                
                izz_node.appendChild(docNode.createTextNode(obj.vec2chr(inertia_vec(3))));
                current_inertia_node.appendChild(izz_node);
                ixy_node = docNode.createElement('Ixy');                
                ixy_node.appendChild(docNode.createTextNode(obj.vec2chr(inertia_vec(4))));
                current_inertia_node.appendChild(ixy_node);
                ixz_node = docNode.createElement('Ixz');                
                ixz_node.appendChild(docNode.createTextNode(obj.vec2chr(inertia_vec(5))));
                current_inertia_node.appendChild(ixz_node);
                iyz_node = docNode.createElement('Iyz');                
                iyz_node.appendChild(docNode.createTextNode(obj.vec2chr(inertia_vec(6))));
                current_inertia_node.appendChild(iyz_node);
                
                % Parent
                current_parent_node = docNode.createElement('parent');
                current_link_node.appendChild(current_parent_node);
                % parent num
                current_parent_num_node = docNode.createElement('num');                
                current_parent_num_node.appendChild(docNode.createTextNode(num2str(obj.links.parentIndex(link_index))));
                current_parent_node.appendChild(current_parent_num_node);
                % location
                current_parent_location_node = docNode.createElement('location');    
                location_vec = cell2mat(obj.links.parent(link_index));
                current_parent_location_node.appendChild(docNode.createTextNode(obj.vec2chr(R*location_vec(1:3))));
                current_parent_node.appendChild(current_parent_location_node);
                
                % Update link_num
                link_num = link_num + 1;
            end       
            
            % Operational space
            operational_spaces_node = docNode.createElement('operational_spaces'); 
            operational_spaces_node.setAttribute('default_operational_set', 'ee_default');
            bodies_system_node.appendChild(operational_spaces_node);
            operational_set_node = docNode.createElement('operational_set'); 
            operational_set_node.setAttribute('id', 'ee_default');
            operational_spaces_node.appendChild(operational_set_node);
            % Operational space position
            position_node = docNode.createElement('position'); 
            position_node.setAttribute('marker_id', '1');
            position_node.setAttribute('name', 'ee_default');
            operational_set_node.appendChild(position_node);
            % link
            link_node = docNode.createElement('link');           
            smallest_child_index = obj.links.Index(obj.links.Index == max(obj.links.Index));
            link_node.appendChild(docNode.createTextNode(obj.vec2chr(smallest_child_index)));
            position_node.appendChild(link_node);
            % offset
            offset_node = docNode.createElement('offset');       
            offset_node.appendChild(docNode.createTextNode(obj.vec2chr(R*obj.links.ee)));
            position_node.appendChild(offset_node);
            % axes
            axes_node = docNode.createElement('axes'); 
            axes_node.setAttribute('active_axes', 'xyz');            
            position_node.appendChild(axes_node);            
            
            filename = sprintf('%s_bodies.xml', obj.model_name);
            xmlwrite(sprintf('%s/%s', obj.model_folder, filename), docNode);
        end
        
        % Create cables xml
        function createCablesXml(obj)            
            docNode = com.mathworks.xml.XMLUtils.createDocument('cables');
            domImpl = docNode.getImplementation();
            doctype = domImpl.createDocumentType('cables', '', '../../../templates/cables.dtd');
            docNode.appendChild(doctype);
            % root node
            cables_node = docNode.getDocumentElement;
            cables_node.setAttribute('default_cable_set', 'DEFAULT');
            % cable set
            cable_set_node = docNode.createElement('cable_set');
            cable_set_node.setAttribute('id','DEFAULT');            
            cables_node.appendChild(cable_set_node);   
            
            % Rotation matrix for fusion STRANGE coordinate system
            if obj.isRotate
                R = [1,0,0;0,0,-1;0,1,0];
            else
                R = eye(3);
            end   
            
            % Each cables
            for i = 1:obj.cables.n_cables - obj.cables.n_invalid_cables  
                % Basic info
                cable_ideal_node = docNode.createElement('cable_ideal');
                cable_ideal_node.setAttribute('attachment_reference', 'com');
                cable_ideal_node.setAttribute('name', obj.cables.Names(i));                
                cable_set_node.appendChild(cable_ideal_node);   
                
                % properties
                properties_node = docNode.createElement('properties');                               
                cable_ideal_node.appendChild(properties_node); 
                force_min_node = docNode.createElement('force_min');     
                force_min_node.appendChild(docNode.createTextNode(obj.vec2chr(obj.f_min)));
                properties_node.appendChild(force_min_node);
                force_max_node = docNode.createElement('force_max');     
                force_max_node.appendChild(docNode.createTextNode(obj.vec2chr(obj.f_max)));
                properties_node.appendChild(force_max_node);
                
                % Attachments
                attachments_node = docNode.createElement('attachments'); 
                cable_ideal_node.appendChild(attachments_node); 
                current_cable = obj.cables.cable_array{i};
                for j = 1:size(current_cable,2)
                    attachment_node = docNode.createElement('attachment'); 
                    % Link num
                    link_node = docNode.createElement('link');     
                    link_num = current_cable{1,j}.link_num;
                    link_node.appendChild(docNode.createTextNode(obj.vec2chr(link_num)));
                    attachment_node.appendChild(link_node);
                    % Location
                    location_node = docNode.createElement('location');     
                    via_point = R*current_cable{1,j}.via_point;
                    location_node.appendChild(docNode.createTextNode(obj.vec2chr(via_point)));
                    attachment_node.appendChild(location_node);
                    
                    attachments_node.appendChild(attachment_node);
                end                   
            end                
            
            filename = sprintf('%s_cables.xml', obj.model_name);
            xmlwrite(sprintf('%s/%s', obj.model_folder, filename), docNode);           
        end
        
        % Create traj xml
        function createTrajXml(obj)            
            docNode = com.mathworks.xml.XMLUtils.createDocument('trajectories');
            domImpl = docNode.getImplementation();
            doctype = domImpl.createDocumentType('trajectories', '', '../../../templates/trajectories.dtd');
            docNode.appendChild(doctype);
            % root node
            trajectories_node = docNode.getDocumentElement;
            
            % joint trajectories
            joint_trajectories_node = docNode.createElement('joint_trajectories');                      
            trajectories_node.appendChild(joint_trajectories_node);   
            
            % operational trajectories
            operational_trajectories_node = docNode.createElement('operational_trajectories');                      
            trajectories_node.appendChild(operational_trajectories_node); 
            
            filename = sprintf('%s_trajectories.xml', obj.model_name);
            xmlwrite(sprintf('%s/%s', obj.model_folder, filename), docNode);          
        end
        
        % Displaying finish message
        function finishMessage(obj)
            CASPR_log.Info('**************************');
            CASPR_log.Info('Conversion completed.');
            CASPR_log.Info(sprintf('Robot Name: %s', obj.model_name));
            CASPR_log.Info('1. Add this model through: CASPR_Model_Manager');
            CASPR_log.Info('2. Overwrite the XML files in the corresponding folder.');
            CASPR_log.Info('**************************');
        end
        
        % Convert vectors to character arrays
        function chr = vec2chr(~, vec)               
            chr = mat2str(vec, 4);
            chr(chr=='[') = '';
            chr(chr==']') = '';
            chr(chr==';') = ' ';
        end
    end
end

