% Convert SDF files to model and cables files
%
% Author        : Dominic Chan
% Created       : 2018
% Description    :
%    This class is a class for the convertion from sdf files to the model
%    files and cables files used in CASPR
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       WARNING                       %
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The current version only supports kinematic chains with ONLY revolute
% joints and no branches. Spatial objects with floating joints are also not
% supported.
% 
% ********          VERY IMPORTANT           **********
%
% Before converting, please make sure the sequence of the links and joints
% in the SDF files are placed in the same way as the kinematic chain.
%
% ********          VERY IMPORTANT           **********

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
            
            % Identify joints
            obj.identifyJoints(model_str);
            
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
        
        % Identify joints
        function identifyJoints(obj, str)
            obj.joints.Names = unique(obj.findAttribute(str, 'joint', 'name'), 'stable');            
            obj.joints.n_joints = length(obj.joints.Names);
            
            pose_vec = obj.findText(str, 'pose');
            joint_pos = cell(1, obj.joints.n_joints);
            
            axes = cell(1, obj.joints.n_joints);
            axes_vec = obj.findText(str, 'xyz');
            
            q_min = cell(1, obj.joints.n_joints);
            q_min_vec = obj.findText(str, 'lower');
            q_max = cell(1, obj.joints.n_joints);
            q_max_vec = obj.findText(str, 'upper');
          
            for i=1:obj.joints.n_joints
                child_com = XmlOperations.StringToVector(pose_vec(1+i));
                joint_offset = XmlOperations.StringToVector(pose_vec(obj.joints.n_joints+1+i));
                joint_pos{i} = child_com + joint_offset;
                axes{i} = XmlOperations.StringToVector(axes_vec(i));
                q_min{i} = XmlOperations.StringToVector(q_min_vec(i));
                q_max{i} = XmlOperations.StringToVector(q_max_vec(i));
            end
            obj.joints.Position = joint_pos;
            obj.joints.Axes = axes;
            obj.joints.q_min = q_min;
            obj.joints.q_max = q_max;
        end
        
        % Identify links
        function identifyLinks(obj, str)
            % Basic info
            parents = obj.findText(str, 'parent');
            children = obj.findText(str, 'child');            
            obj.links.Names = union(parents, children, 'stable');
            obj.links.Names = obj.links.Names(2:end);
            obj.links.n_links = length(obj.links.Names);
            obj.links.Index = 1:1:obj.links.n_links;
            
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
            com_vec = obj.findText(str, 'pose');
            
            % Loop through the links to get info
            for i=1:obj.links.n_links
                mass{i} = XmlOperations.StringToVector(mass_vec(i+1));
                inertia{i} = [XmlOperations.StringToVector(ixx_vec(i+1));
                    XmlOperations.StringToVector(iyy_vec(i+1));
                    XmlOperations.StringToVector(izz_vec(i+1));
                    XmlOperations.StringToVector(ixy_vec(i+1));
                    XmlOperations.StringToVector(ixz_vec(i+1));
                    XmlOperations.StringToVector(iyz_vec(i+1))];
                
                abs_com = XmlOperations.StringToVector(com_vec(i+1));
                com_pos{i} = abs_com - obj.joints.Position{i};
                if i ~= obj.links.n_links
                    end_pos{i} = obj.joints.Position{i+1} - obj.joints.Position{i};
                else
                    end_pos{i} = 2*com_pos{i};
                end
                if i ~= 1
                    % Use the previous end position as the parent
                    parent_pos{i} = end_pos{i-1};
                else
                    fixed_com = XmlOperations.StringToVector(com_vec(1));
                    parent_pos{i} = obj.joints.Position{1} - fixed_com;
                end
            end
            
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
                % Default ee
                ee_pos = end_pos{end};
                obj.links.ee = ee_pos(1:3);
            end
            
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
                
                current_cable = cell(1,0);
                % Loop through each link
                for j=1:length(start_link_idx)                    
                    starting_link_row = mod(start_link_idx(j), size(cable_str,1));
                    ending_link_row = mod(end_link_idx(j), size(cable_str,1));
                    link_str = cable_str(starting_link_row:ending_link_row,:);
                    % Name
                    link_name = obj.findAttribute(link_str, 'link', 'name'); 
                    link_num = find(contains(obj.links.Names, link_name));
                    if isempty(link_num)
                        link_num = 0;
                    end
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
            
            for i=1:obj.links.n_links
                % Basic info
                current_link_node = docNode.createElement('link_rigid');
                current_link_node.setAttribute('num', num2str(obj.links.Index(i)));
                current_link_node.setAttribute('name', obj.links.Names(i));
                links_node.appendChild(current_link_node);
                
                % Joint
                current_joint_node = docNode.createElement('joint');
                current_joint_node.setAttribute('type', 'R_AXIS');
                current_joint_node.setAttribute('q_initial', '0.0');
                q_min_vec = cell2mat(obj.joints.q_min(i));
                current_joint_node.setAttribute('q_min', obj.vec2chr(q_min_vec));
                q_max_vec = cell2mat(obj.joints.q_max(i));
                current_joint_node.setAttribute('q_max', obj.vec2chr(q_max_vec));
                axis_vec = cell2mat(obj.joints.Axes(i)); 
                current_joint_node.setAttribute('axis', obj.vec2chr(R*axis_vec));
                current_link_node.appendChild(current_joint_node);
                
                % Physical
                current_physical_node = docNode.createElement('physical');
                current_link_node.appendChild(current_physical_node);
                
                % Mass
                current_mass_node = docNode.createElement('mass');                
                mass_vec = cell2mat(obj.links.Mass(i));
                current_mass_node.appendChild(docNode.createTextNode(obj.vec2chr(mass_vec)));
                current_physical_node.appendChild(current_mass_node);
                
                % COM
                current_com_node = docNode.createElement('com_location');
                com_vec = cell2mat(obj.links.com(i));
                current_com_node.appendChild(docNode.createTextNode(obj.vec2chr(R*com_vec(1:3))));
                current_physical_node.appendChild(current_com_node);
                
                % END
                current_end_node = docNode.createElement('end_location');
                end_vec = cell2mat(obj.links.end(i));
                current_end_node.appendChild(docNode.createTextNode(obj.vec2chr(R*end_vec(1:3))));
                current_physical_node.appendChild(current_end_node);
                
                % Inertia
                current_inertia_node = docNode.createElement('inertia');
                current_inertia_node.setAttribute('ref', 'com');
                current_physical_node.appendChild(current_inertia_node);
                
                inertia_vec = cell2mat(obj.links.Inertia(i));                
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
                current_parent_num_node.appendChild(docNode.createTextNode(num2str(i-1)));
                current_parent_node.appendChild(current_parent_num_node);
                % location
                current_parent_location_node = docNode.createElement('location');    
                location_vec = cell2mat(obj.links.parent(i));
                current_parent_location_node.appendChild(docNode.createTextNode(obj.vec2chr(R*location_vec(1:3))));
                current_parent_node.appendChild(current_parent_location_node);
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
            link_node.appendChild(docNode.createTextNode(obj.vec2chr(obj.links.n_links)));
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
%             type(sprintf('%s/%s', obj.model_folder, filename));
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
            for i = 1:obj.cables.n_cables   
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
%             type(sprintf('%s/%s', obj.model_folder, filename));            
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
%             type(sprintf('%s/%s', obj.model_folder, filename));            
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

