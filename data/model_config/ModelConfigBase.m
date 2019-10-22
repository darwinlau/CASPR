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
        bodyPropertiesFilename      % Filename for the body properties
        cablesPropertiesFilename    % Filename for the cable properties
        trajectoriesFilename        % Filename for the trajectories
        
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
        jointTrajectoryNamesList        % List of names of trajectories (joint space)
        operationalTrajectoryNamesList  % List of names of trajectories (operational space)
    end
    
    properties (Access = private)
        root_folder                 % The root folder for the models
        
        bodiesXmlObj                % The DOMNode object for body props
        cablesXmlObj                % The DOMNode object for cable props
        trajectoriesXmlObj          % The DOMNode for trajectory props
    end
    
    methods
        % Constructor for the ModelConfig class. This builds the xml
        % objects.
        function c = ModelConfigBase(robot_name, folder, list_file)
            c.root_folder = [CASPR_configuration.LoadModelConfigPath(), folder];
            c.robotName = robot_name;
                        
            % Check the type of enum and open the master list
            fid = fopen([c.root_folder, list_file]);
            
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
                    c.modelFolderPath           = [c.root_folder, cdpr_folder];
                    c.bodyPropertiesFilename    = [c.root_folder, cdpr_folder, char(cell_array{3}{i})];
                    c.cablesPropertiesFilename  = [c.root_folder, cdpr_folder, char(cell_array{4}{i})];
                    c.trajectoriesFilename      = [c.root_folder, cdpr_folder, char(cell_array{5}{i})];
                    fclose(fid);
                    status_flag = 0;
                    break;
                end
            end
            if(status_flag)
                CASPR_log.Error(sprintf('Robot model ''%s'' is not defined', robot_name));
            end
            
            % Make sure all the filenames that are required exist
            CASPR_log.Assert(exist(c.bodyPropertiesFilename, 'file') == 2, 'Body properties file does not exist.');
            CASPR_log.Assert(exist(c.cablesPropertiesFilename, 'file') == 2, 'Cable properties file does not exist.');
            CASPR_log.Assert(exist(c.trajectoriesFilename, 'file') == 2, 'Trajectories file does not exist.');
            % Read the XML file to an DOM XML object
            c.bodiesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.bodyPropertiesFilename);
            c.cablesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.cablesPropertiesFilename);
            c.trajectoriesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.trajectoriesFilename);
            
            % Loads the bodiesModel and cable set to be used for the trajectory loading
            bodies_xmlobj = c.getBodiesPropertiesXmlObj();
            
            default_cable_set_id = char(c.cablesXmlObj.getDocumentElement.getAttribute('default_cable_set'));
            CASPR_log.Assert(~isempty(default_cable_set_id), '''default_cable_set'' must be specified in the cables XML configuration file.');
            c.defaultCableSetId = default_cable_set_id;
            
            cableset_xmlobj = c.getCableSetXmlObj(c.defaultCableSetId);
            
            if(c.bodiesXmlObj.getElementsByTagName('operational_spaces').getLength ~= 0 && ~isempty(char(c.bodiesXmlObj.getElementsByTagName('operational_spaces').item(0).getAttribute('default_operational_set'))))
                c.defaultOperationalSetId = char(c.bodiesXmlObj.getElementsByTagName('operational_spaces').item(0).getAttribute('default_operational_set'));
                operationalset_xmlobj = c.getOperationalSetXmlObj(c.defaultOperationalSetId);
            else 
                operationalset_xmlobj = [];
            end
            
            % At this stage load the default system model so trajectories
            % can be loaded.
            sysModel = SystemModel.LoadXmlObj(c.robotName, bodies_xmlobj, cableset_xmlobj, operationalset_xmlobj, ModelModeType.DEFAULT, ModelOptions());
            
            c.bodiesModel = sysModel.bodyModel;
            c.displayRange = XmlOperations.StringToVector(char(bodies_xmlobj.getAttribute('display_range')))';
            c.viewAngle = XmlOperations.StringToVector(char(bodies_xmlobj.getAttribute('view_angle')))';
        end
                
        function [sysModel] = getModel(obj, cable_set_id, operational_space_id, model_mode, model_options)
            bodies_xmlobj = obj.getBodiesPropertiesXmlObj();
            cableset_xmlobj = obj.getCableSetXmlObj(cable_set_id);
            
            if nargin < 3 || isempty(operational_space_id)
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
                % First check if a new compilation is required
                
                % If so, then perform compilation through the symbolic mode
                
                % Finally, create a SystemModel object (in compiled mode) to return
                
                
                model_config_path = CASPR_configuration.LoadModelConfigPath();
                compile_file_folder = [model_config_path, '\tmp_compilations'];
                
                    % Start Compilations...
                    warning('off','all');  
                    CASPR_log.Warn('Current version of compiled mode does not support changes in active <-> passive cables.');
                    CASPR_log.Warn('Compilation needs a long time. Please wait...');                        
                    warning('on','all');  
                    CASPR_log.Info('Preparation work...');
                    sysModelSym = SystemModel.LoadXmlObj(obj.robotName, bodies_xmlobj, cableset_xmlobj, op_space_set_xmlobj, ModelModeType.SYMBOLIC, model_options);
                    sysModelSym.compile(compile_file_folder);
                    
                    sysModel = SystemModel.LoadXmlObj(obj.robotName, bodies_xmlobj, cableset_xmlobj, op_space_set_xmlobj, ModelModeType.COMPILED, model_options);
                               
%                 end     
%             elseif model_mode == ModelModeType.CUSTOM
%                 % Use this two step approach for now
%                 % If op space can also be updated in custom mode,
%                 % then we can run CUSTOM mode at the first run                
%                 
%                 % Run DEFAULT mode first
%                 sysModel = SystemModel.LoadXmlObj(bodies_xmlobj,cableset_xmlobj,ModelModeType.DEFAULT);
%                 % Operational
%                 if ~isempty(operational_space_id)
%                     operationalset_xmlobj = obj.getOperationalSetXmlObj(operational_space_id);
%                     sysModel.loadOperationalXmlObj(operationalset_xmlobj);
%                     sysModel.bodyModel.updateOperationalSpace();
%                 end              
%             elseif model_mode == ModelModeType.COMPILED_AUTO
%                 model_config_path = CASPR_configuration.LoadModelConfigPath();
%                 library_name = ModelConfigBase.GetAutoCompiledLibraryName(obj.robotName, cable_set_id);
%                 
%                 
%                 % Step 1: Check if the robot needs to be compiled first
%                 
%                 % Step 2: Compile .m files from symbolic to temp compilations folder
%                 compile_file_folder = [model_config_path, '\tmp_compilations'];
%                 
%                 % Remove all the existing compiled files to ensure all compiled
%                 % files are up-to-date
%                 if exist(compile_file_folder, 'dir')
%                     warning('off','all');  
%                     try
%                         if libisloaded(library_name)
%                             unloadlibrary(library_name);
%                         end
%                         rmpath(genpath(compile_file_folder));
%                         rmdir(compile_file_folder, 's');
%                         CASPR_log.Info('Previously compiled files are removed.');
%                     catch
%                         CASPR_log.Warn('You might not have the permission to remove previously compiled files');
%                         CASPR_log.Warn('Please check your permission before running COMPILED mode.');
%                         return;
%                     end
%                     warning('on','all');
%                 else
%                     % Make folders
%                     mkdir(compile_file_folder);  
%                 end
%                 
%                 % Start Compilations...
%                 warning('off','all');
%                 CASPR_log.Warn('Current version of compiled mode does not support changes in active <-> passive cables.');
%                 CASPR_log.Warn('Compilation needs a long time. Please wait...');
%                 warning('on','all');
%                 CASPR_log.Info('Preparation work...');
%                 
%                 
%                 
%                 sysModelSym = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj, ModelModeType.SYMBOLIC);
%                 sysModelSym.compile(compile_file_folder);
%                     
%                 % Operational
% %                 if ~isempty(operational_space_id)
% %                     operationalset_xmlobj = obj.getOperationalSetXmlObj(obj.defaultOperationalSetId);
% %                     sysModel.loadOperationalXmlObj(operationalset_xmlobj);
% %                     sysModel.bodyModel.updateOperationalSpace();
% %                 end
%                 
%                 
%                 % Step 3: Compile C version of the .m code
%                 build_folder = [compile_file_folder, '/c_build'];
%                 filesList = dir([compile_file_folder, '/**/', '*.m']);
%                 
%                 source_files = cell(1, length(filesList));
%                 
%                 for i = 1:length(filesList)
%                     source_files{i} = [filesList(i).folder, '\', filesList(i).name];
%                 end
%                 
%                 sysModelSym.numDofs
%                 input_data = {zeros(sysModelSym.numDofs,1),zeros(sysModelSym.numDofs,1),zeros(sysModelSym.numDofs,1),zeros(sysModelSym.numDofs,1)};
%                 
%                 % Config
%                 code_config = coder.config('dll');
%                 %code_config.IncludeTerminateFcn = false;
%                 code_config.SupportNonFinite = false;
%                 code_config.SaturateOnIntegerOverflow = false;
%                 code_config.GenerateExampleMain = 'DoNotGenerate';
%                 code_config.TargetLang = 'C';
%                 code_config.FilePartitionMethod = 'SingleFile';
%                 
%                 % Code generation
%                 
%                 str = ['codegen -d ', build_folder, ' -o ', library_name, ' -config code_config '];
%                 for i = 1:length(source_files)
%                     str = [str, source_files{i},' -args input_data '];
%                 end
%                 eval(str)
%                 % Remove unnecessary stuff
%                 rmdir([build_folder,'/examples']);
%                 delete([build_folder,'/*.mat']);
%                 
%                 addpath(genpath(build_folder));
                
%                 warning('off','all');  
%                 if (~libisloaded(library_name))
%                     loadlibrary(library_name);
%                 end
%                 warning('on','all');
%                 
%                 sysModel = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj, ModelModeType.COMPILED_AUTO);
            else
                % DEFAULT || SYMBOLIC 
                sysModel = SystemModel.LoadXmlObj(obj.robotName, bodies_xmlobj, cableset_xmlobj, op_space_set_xmlobj, model_mode, model_options);
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
            cablesetsObj = obj.cablesXmlObj.getElementsByTagName('cables').item(0).getElementsByTagName('cable_set');
            cableset_str = cell(1,cablesetsObj.getLength);
            % Extract the identifies from the cable sets
            for i =1 :cablesetsObj.getLength
                cablesetObj = cablesetsObj.item(i-1);
                cableset_str{i} = char(cablesetObj.getAttribute('id'));
            end
        end
        
        function trajectories_str = getJointTrajectoriesList(obj)
            if (~isempty(obj.trajectoriesXmlObj.getElementsByTagName('joint_trajectories').item(0)))
                trajectories_str = GUIOperations.XmlObj2StringCellArray(obj.trajectoriesXmlObj.getElementsByTagName('joint_trajectories').item(0).getChildNodes,'id');
            else
                trajectories_str = {};
            end
        end
        
        function trajectories_str = getOperationalTrajectoriesList(obj)
            if (~isempty(obj.trajectoriesXmlObj.getElementsByTagName('operational_trajectories').item(0)))
                trajectories_str = GUIOperations.XmlObj2StringCellArray(obj.trajectoriesXmlObj.getElementsByTagName('operational_trajectories').item(0).getChildNodes,'id');
            else
                trajectories_str = {};
            end
        end
    end
    
    % Dependent variables
    methods
        function value = get.cableSetNamesList(obj)
            value = obj.getCableSetList();
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
            v_temp = obj.bodiesXmlObj.getElementsByTagName('links');
            CASPR_log.Assert(v_temp.getLength == 1,'1 links tag should be specified');
            v = v_temp.item(0);
        end
        
        % Gets the cable set properties xml object
        function v = getCableSetXmlObj(obj, id)
            v = obj.cablesXmlObj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the cables XML file', id));
        end
        
        % Get the trajectory xml object
        function v = getJointTrajectoryXmlObj(obj, id)
            v = obj.trajectoriesXmlObj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the trajectories XML file', id));
        end
        
        function v = getOperationalTrajectoryXmlObj(obj, id)
            v = obj.trajectoriesXmlObj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the trajectories XML file', id));
        end
        
        % Get the operational space xml object
        function v = getOperationalSetXmlObj(obj, id)
            v_temp = obj.bodiesXmlObj.getElementsByTagName('operational_spaces');
            CASPR_log.Assert(v_temp.getLength == 1,'1 operational space tag should be specified');
            v = obj.bodiesXmlObj.getElementById(id);
            CASPR_log.Assert(~isempty(v), sprintf('Id ''%s'' does not exist in the bodies XML file', id));
        end
    end
    
    methods (Static)
        function library_name = GetAutoCompiledLibraryName(robot_name, cable_set_id)
            library_name = [robot_name, '_', cable_set_id];
            library_name = strrep(library_name, ' ', '_');
            library_name = strrep(library_name, '-', '_');
        end
    end
end