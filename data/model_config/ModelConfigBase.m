% Base class for the model configuration
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :
%    This is the base class for all of the model configurations within
%    CASPR. The CDPR information are stored as XML files and new robots
%    must be added to the ModelConfigType and models_list.csv to be
%    accessible.
classdef (Abstract) ModelConfigBase < handle
    properties (SetAccess = private)
        bodiesPropertiesFilepath                % File path for the body properties
        operationalSpacesPropertiesFilepath     % File path for the operational space properties
        cablesPropertiesFilepath                % File path for the cable properties
        trajectoriesFilepath                    % File path for the trajectories
        
        modelFolderPath             % Path of folder for the model
        
        bodiesModel                 % Stores the SystemModelBodies object for the robot model
        displayRange                % The axis range to display the robot
        viewAngle                   % The angle for which the model should be viewed at
        defaultCableSetId           % ID for the default cable set to display first
        defaultOperationalSetId     % ID for the default operational set to display first
        
        robotNamesList              % List of names of robots as cell array of strings
        robotName                   % Name of the robot
    end
    
    properties (Dependent)
        cableSetNamesList               % List of names of cable sets
        operationalSpaceSetNamesList    % List of names of operational space sets
        jointTrajectoryNamesList        % List of names of trajectories (joint space)
        operationalTrajectoryNamesList  % List of names of trajectories (operational space)
    end
        
    properties (Access = private)
        root_folder_path            % The root folder for the models
        
        bodies_xml_obj                  % The DOMNode object for body props
        cables_xml_obj                  % The DOMNode object for cable props
        trajectories_xml_obj            % The DOMNode for trajectory props
        operational_spaces_xml_obj      % The DOMNode for operational spaces
    end
    
    properties (Constant)
        COMPILE_RECORD_FILENAME = 'compile_record.txt'
        COMPILE_RECORD_BODIES_FILENAME = 'compile_record_bodies.txt'
        COMPILE_RECORD_CABLES_FILENAME = 'compile_record_cables.txt'
        COMPILE_RECORD_OPERATIONAL_SPACES_FILENAME = 'compile_record_operationalspaces.txt'
        DEFAULT_COMPILED_FOLDER = 'tmp_compilations'
    end
    
    methods
        % Constructor for the ModelConfig class. This builds the xml
        % objects.
        function c = ModelConfigBase(robot_name, folder, list_file)
            c.root_folder_path = [CASPR_configuration.LoadModelConfigPath(), folder];
            c.robotName = robot_name;
                        
            % Check the type of enum and open the master list
            fid = fopen([c.root_folder_path, list_file]);
            
            % Determine the Filenames
            % Load the contents
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            num_robots = length(cell_array{1});
            c.robotNamesList = cell(1, num_robots);
            status_flag = 1;
            % Store the robot names
            for i = 1:num_robots
                c.robotNamesList{i} = char(cell_array{1}{i});
            end
            % Loop through until the right line of the list is found
            for i = 1:num_robots
                if(strcmp(c.robotNamesList{i},robot_name))
                    cdpr_folder                 = char(cell_array{2}{i});
                    c.modelFolderPath           = [c.root_folder_path, cdpr_folder];
                    c.bodiesPropertiesFilepath    = [c.root_folder_path, cdpr_folder, char(cell_array{3}{i})];
                    c.cablesPropertiesFilepath  = [c.root_folder_path, cdpr_folder, char(cell_array{4}{i})];
                    c.trajectoriesFilepath      = [c.root_folder_path, cdpr_folder, char(cell_array{5}{i})];
                    c.operationalSpacesPropertiesFilepath    = [c.root_folder_path, cdpr_folder, char(cell_array{6}{i})];
                    fclose(fid);
                    status_flag = 0;
                    break;
                end
            end
            if(status_flag)
                CASPR_log.Error(sprintf('Robot model ''%s'' is not defined', robot_name));
            end
            
            % Make sure all the filenames that are required exist
            CASPR_log.Assert(exist(c.bodiesPropertiesFilepath, 'file') == 2, ['Body properties file does not exist: ', c.bodiesPropertiesFilepath]);
            CASPR_log.Assert(exist(c.cablesPropertiesFilepath, 'file') == 2, ['Cable properties file does not exist: ', c.cablesPropertiesFilepath]);
            CASPR_log.Assert(exist(c.operationalSpacesPropertiesFilepath, 'file') == 2, ['Operational spaces properties file does not exist: ', c.operationalSpacesPropertiesFilepath]);
            CASPR_log.Assert(exist(c.trajectoriesFilepath, 'file') == 2, ['Trajectories file does not exist: ', c.trajectoriesFilepath]);
            % Read the XML file to an DOM XML object
            c.bodies_xml_obj =  XmlOperations.XmlReadRemoveIndents(c.bodiesPropertiesFilepath);
            c.cables_xml_obj =  XmlOperations.XmlReadRemoveIndents(c.cablesPropertiesFilepath);
            c.trajectories_xml_obj =  XmlOperations.XmlReadRemoveIndents(c.trajectoriesFilepath);
            c.operational_spaces_xml_obj = XmlOperations.XmlReadRemoveIndents(c.operationalSpacesPropertiesFilepath);
            
            % Loads the bodiesModel and cable set to be used for the trajectory loading
            bodies_xmlobj = c.getBodiesPropertiesXmlObj();
            
            % Loads the default cable set
            default_cable_set_id = char(c.cables_xml_obj.getDocumentElement.getAttribute('default_cable_set'));
            CASPR_log.Assert(~isempty(default_cable_set_id), '''default_cable_set'' must be specified in the cables XML configuration file.');
            c.defaultCableSetId = default_cable_set_id;
            cableset_xmlobj = c.getCableSetXmlObj(c.defaultCableSetId);
            
            % Loads the default operational space set
            c.defaultOperationalSetId = char(c.operational_spaces_xml_obj.getDocumentElement.getAttribute('default_operational_set'));
            if(~isempty(c.defaultOperationalSetId))
                operationalset_xmlobj = c.getOperationalSetXmlObj(c.defaultOperationalSetId);
            else 
                operationalset_xmlobj = [];
            end
            
            % At this stage load the default system model so trajectories
            % can be loaded.
            sysModel = SystemModel.LoadXmlObj(c.robotName, bodies_xmlobj, default_cable_set_id, cableset_xmlobj, c.defaultOperationalSetId, operationalset_xmlobj, ModelModeType.DEFAULT, ModelOptions());
            
            c.bodiesModel = sysModel.bodyModel;
            c.displayRange = XmlOperations.StringToVector(char(bodies_xmlobj.getAttribute('display_range')))';
            c.viewAngle = XmlOperations.StringToVector(char(bodies_xmlobj.getAttribute('view_angle')))';
        end
                
        function [sysModel] = getModel(obj, cable_set_id, operational_space_id, model_mode, model_options)
            bodies_xmlobj = obj.getBodiesPropertiesXmlObj();
            cableset_xmlobj = obj.getCableSetXmlObj(cable_set_id);
            
            if nargin < 3 || isempty(operational_space_id)
                operational_space_id = [];
                op_space_set_xmlobj = [];
            else
                op_space_set_xmlobj = obj.getOperationalSetXmlObj(operational_space_id);
            end
            if nargin < 4 || isempty(model_mode)
                model_mode = ModelModeType.DEFAULT;
            end
            if (nargin < 5 || isempty(model_options))
                model_options = ModelOptions();
            end

            if model_mode == ModelModeType.COMPILED
                base_compile_folder = ModelConfigBase.GetDefaultCompiledFolder(obj.robotName);

                % Check if a new compilation is required
                [compile_bodies, compile_cables, compile_opspaces] = obj.checkRequireCompile(cable_set_id, operational_space_id, base_compile_folder);
                
                if(compile_bodies || compile_cables || compile_opspaces)
                    CASPR_log.Info('Creating symbolic model');
                    % Create a symbolic model
                    sysModelSym = SystemModel.LoadXmlObj(obj.robotName, bodies_xmlobj, cable_set_id, cableset_xmlobj, operational_space_id, op_space_set_xmlobj, ModelModeType.SYMBOLIC, model_options);
                    CASPR_log.Info('Compiling symbolic model');
                    % Do the .m compile
                    sysModelSym.compile(base_compile_folder, obj);
                end
                sysModel = SystemModel.LoadXmlObj(obj.robotName, bodies_xmlobj, cable_set_id, cableset_xmlobj, operational_space_id, op_space_set_xmlobj, ModelModeType.COMPILED, model_options);
            else
                % DEFAULT || SYMBOLIC 
                sysModel = SystemModel.LoadXmlObj(obj.robotName, bodies_xmlobj, cable_set_id, cableset_xmlobj, operational_space_id, op_space_set_xmlobj, model_mode, model_options);
            end
            obj.bodiesModel = sysModel.bodyModel;
        end        
        
        % Getting the joint and operational space trajectories
        function [traj] = getJointTrajectory(obj, trajectory_id)
            traj_xmlobj = obj.getJointTrajectoryXmlObj(trajectory_id);
            traj = JointTrajectory.LoadXmlObj(traj_xmlobj, obj.bodiesModel, obj);
        end
        
        function [traj] = getOperationalTrajectory(obj, trajectory_id)
            traj_xmlobj = obj.getOperationalTrajectoryXmlObj(trajectory_id);
            traj = OperationalTrajectory.LoadXmlObj(traj_xmlobj, obj.bodiesModel, obj);
        end
        
        function cableset_str = getCableSetList(obj)
            cablesetsObj = obj.cables_xml_obj.getElementsByTagName('cables').item(0).getElementsByTagName('cable_set');
            cableset_str = cell(1,cablesetsObj.getLength);
            % Extract the identifies from the cable sets
            for i =1 :cablesetsObj.getLength
                cablesetObj = cablesetsObj.item(i-1);
                cableset_str{i} = char(cablesetObj.getAttribute('id'));
            end
        end
        
        function opset_str = getOperationalSpaceSetList(obj)
            opsetsObj = obj.operational_spaces_xml_obj.getElementsByTagName('operational_spaces').item(0).getElementsByTagName('operational_set');
            opset_str = cell(1,opsetsObj.getLength);
            % Extract the identifies from the cable sets
            for i =1 :opsetsObj.getLength
                opsetObj = opsetsObj.item(i-1);
                opset_str{i} = char(opsetObj.getAttribute('id'));
            end
        end
        
        function trajectories_str = getJointTrajectoriesList(obj)
            if (~isempty(obj.trajectories_xml_obj.getElementsByTagName('joint_trajectories').item(0)))
                trajectories_str = GUIOperations.XmlObj2StringCellArray(obj.trajectories_xml_obj.getElementsByTagName('joint_trajectories').item(0).getChildNodes,'id');
            else
                trajectories_str = {};
            end
        end
        
        function trajectories_str = getOperationalTrajectoriesList(obj)
            if (~isempty(obj.trajectories_xml_obj.getElementsByTagName('operational_trajectories').item(0)))
                trajectories_str = GUIOperations.XmlObj2StringCellArray(obj.trajectories_xml_obj.getElementsByTagName('operational_trajectories').item(0).getChildNodes,'id');
            else
                trajectories_str = {};
            end
        end
        
        function [compile_bodies, compile_cables, compile_opspaces] = checkRequireCompile(obj, cableset_id, opspace_id, base_folder)
            compile_bodies_dtstamp_file = [base_folder, '\', obj.COMPILE_RECORD_BODIES_FILENAME];
            compile_cables_dtstamp_file = [base_folder, '\', obj.COMPILE_RECORD_CABLES_FILENAME];
            compile_opspaces_dtstamp_file = [base_folder, '\', obj.COMPILE_RECORD_OPERATIONAL_SPACES_FILENAME];
            
            [~, ~, ~, ~, bodies_folder, cableset_folder, opset_folder, ~, ~, ~, ~] = ModelConfigBase.ConstructCompiledLibraryNames(obj.robotName, cableset_id, opspace_id, base_folder);
            
            % Get datetime of last modified from the bodies.xml and
            % cables.xml
            bodyfile = dir(obj.bodiesPropertiesFilepath);
            opspacefile = dir(obj.operationalSpacesPropertiesFilepath);
            cablefile = dir(obj.cablesPropertiesFilepath);
            
            % Get current datetime and timezone information
            curr_date = datetime('now', 'TimeZone', 'local');
            
            compile_bodies = false;
            compile_cables = false;
            compile_opspaces = false;
            
            % Check if the bodies folder should be compiled
            if (~exist(compile_bodies_dtstamp_file, 'file') || ~exist(bodies_folder, 'dir'))
                compile_bodies = true;
                compile_cables = true;
                compile_opspaces = true;
            else
                [datetime_out, timezone] = ModelConfigBase.GetCompiledDatetime(compile_bodies_dtstamp_file);
                % If the timezone from file is not the system, maybe a change
                % in system times occured, recompile anyway for safety
                if (strcmp(curr_date.TimeZone, timezone) == false)
                    compile_bodies = true;
                    compile_cables = true;
                    compile_opspaces = true;
                elseif (bodyfile.date > datetime_out)
                    compile_bodies = true;
                    compile_cables = true;
                    compile_opspaces = true;
                end
            end
            
            % Check if the cables folder should be compiled
            if (~exist(compile_cables_dtstamp_file, 'file') || ~exist(cableset_folder, 'dir'))
                compile_cables = true;
            else
                [datetime_out, timezone] = ModelConfigBase.GetCompiledDatetime(compile_cables_dtstamp_file);
                % If the timezone from file is not the system, maybe a change
                % in system times occured, recompile anyway for safety
                if (curr_date.TimeZone ~= timezone)
                    compile_cables = true;
                elseif (cablefile.date > datetime_out)
                    compile_cables = true;
                end
            end
            
            % Check if the operational spaces folder should be compiled
            if (~isempty(opspace_id))
                if (~exist(compile_opspaces_dtstamp_file, 'file') || ~exist(opset_folder, 'dir'))
                    compile_opspaces = true;
                else
                    [datetime_out, timezone] = ModelConfigBase.GetCompiledDatetime(compile_opspaces_dtstamp_file);
                    % If the timezone from file is not the system, maybe a change
                    % in system times occured, recompile anyway for safety
                    if (curr_date.TimeZone ~= timezone)
                        compile_opspaces = true;
                    elseif (opspacefile.date > datetime_out)
                        compile_opspaces = true;
                    end
                end
            else
                compile_opspaces = false;
            end
        end
    end
    
    % Dependent variables
    methods
        function value = get.cableSetNamesList(obj)
            value = obj.getCableSetList();
        end
        
        function value = get.operationalSpaceSetNamesList(obj)
            value = obj.getOperationalSpaceSetList();
        end
        
        function value = get.jointTrajectoryNamesList(obj)
            value = obj.getJointTrajectoriesList();
        end
        
        function value = get.operationalTrajectoryNamesList(obj)
            value = obj.getOperationalTrajectoriesList();
        end
    end
    
    methods (Access = private)
        % Gets the body properties xml object
        function v = getBodiesPropertiesXmlObj(obj)
            v_temp = obj.bodies_xml_obj.getElementsByTagName('links');
            CASPR_log.Assert(v_temp.getLength == 1,'Only one <links> tag should be specified');
            v = v_temp.item(0);
        end
        
        % Gets the cable set properties xml object
        function v = getCableSetXmlObj(obj, id)
            v = obj.cables_xml_obj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the cables XML file', id));
        end
        
        % Get the trajectory xml object
        function v = getJointTrajectoryXmlObj(obj, id)
            v = obj.trajectories_xml_obj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the trajectories XML file', id));
        end
        
        function v = getOperationalTrajectoryXmlObj(obj, id)
            v = obj.trajectories_xml_obj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the trajectories XML file', id));
        end
        
        % Get the operational space xml object
        function v = getOperationalSetXmlObj(obj, id)
            v = obj.operational_spaces_xml_obj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the operational spaces XML file', id));
        end
    end
    
    methods (Static)
        function [bodies_lib_name, cableset_lib_name, opset_lib_name, ...
                cpp_lib_name, bodies_folder, cableset_folder, opset_folder, cpp_folder, m_folder, ...
                cables_base_folder, opset_base_folder] = ConstructCompiledLibraryNames(robot_name, cable_set_id, op_space_id, base_folder)
            
            if (nargin < 4)
                base_folder = ModelConfigBase.GetDefaultCompiledFolder(robot_name);
            end
            
            % First process the invalid or unwanted characters
            robot_name = strrep(strrep(robot_name, ' ', '_'), '-', '_');
            cable_set_id = strrep(strrep(cable_set_id, ' ', '_'), '-', '_');
            
            if (~isempty(op_space_id))
                op_space_id = strrep(strrep(op_space_id, ' ', '_'), '-', '_');
            end               
            
            bodies_lib_name = robot_name;
            cableset_lib_name = [robot_name, '_', cable_set_id];
            cpp_lib_name = cableset_lib_name;
            
            m_folder = [base_folder, '\m'];
            
            bodies_folder = [m_folder, '\Bodies'];
            cables_base_folder = [m_folder, '\Cables'];
            cableset_folder = [cables_base_folder, '\', cable_set_id];
            
            if (~isempty(op_space_id))
                opset_lib_name = [robot_name, '_', op_space_id];
                cpp_lib_name = [cableset_lib_name, '_', op_space_id];
                opset_base_folder = [base_folder, '\m\OperationalSpaces'];
                opset_folder = [opset_base_folder, '\', op_space_id];
            else 
                opset_lib_name = '';
                opset_base_folder = '';
                opset_folder = '';
            end
            
            cpp_folder = [base_folder, '\cpp\', cpp_lib_name];            
        end
        
        function WriteCompileRecordFile(compile_file_folder, filename)
            fid = fopen([compile_file_folder, '\', filename], 'w');
            dt = datetime('now', 'TimeZone', 'local');
            fprintf(fid, ['compiledtime,', datestr(dt), ',', dt.TimeZone]);
            fclose(fid);
        end
        
        function [datetime_out, timezone] = GetCompiledDatetime(file)
            CASPR_log.Assert(exist(file, 'file'), 'Compiled timestamp file does not exist');
            try
                % Get data from the compiled record file
                fid = fopen(file, 'r');
                line = fgetl(fid);
                fclose(fid);
                split_str = strsplit(line, ',');
                id_str = split_str{1};
                datetime_out = datetime(split_str{2});
                timezone = split_str{3};
                CASPR_log.Assert(id_str == 'compiledtime', 'Invalid compiled record file');
            catch
                CASPR_log.Error('Invalid compiled record file');
            end
        end
        
        function path = GetDefaultCompiledFolder(robot_name)
            robot_name_path = strrep(strrep(robot_name, ' ', '_'), '-', '_');
            path = [CASPR_configuration.LoadModelConfigPath(), '\tmp_compilations\', robot_name_path];
        end
    end
end