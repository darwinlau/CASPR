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
    end
    
    properties (Access = private)
        root_folder                 % The root folder for the models
        
        robotName                   % Name of the robot
        bodiesXmlObj                % The DOMNode object for body props
        cablesXmlObj                % The DOMNode object for cable props
        trajectoriesXmlObj          % The DOMNode for trajectory props
    end
    
    methods
        % Constructor for the ModelConfig class. This builds the xml
        % objects.
        function c = ModelConfigBase(type_string, folder, list_file)
            c.root_folder = [CASPR_configuration.LoadModelConfigPath(), folder];
            c.robotName = type_string;
                        
            % Check the type of enum and open the master list
            fid = fopen([c.root_folder, list_file]);
            
            % Determine the Filenames
            % Load the contents
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            i_length = length(cell_array{1});
            status_flag = 1;
            % Loop through until the right line of the list is found
            for i = 1:i_length
                if(strcmp(char(cell_array{1}{i}),type_string))
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
                CASPR_log.Error(sprintf('Robot model ''%s'' is not defined', type_string));
            end
            
            % Make sure all the filenames that are required exist
            assert(exist(c.bodyPropertiesFilename, 'file') == 2, 'Body properties file does not exist.');
            assert(exist(c.cablesPropertiesFilename, 'file') == 2, 'Cable properties file does not exist.');
            assert(exist(c.trajectoriesFilename, 'file') == 2, 'Trajectories file does not exist.');
            % Read the XML file to an DOM XML object
            c.bodiesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.bodyPropertiesFilename);
            c.cablesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.cablesPropertiesFilename);
            c.trajectoriesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.trajectoriesFilename);
            % If the operational space filename is specified then check and load it
%             if (~isempty(c.opFilename))
%                 assert(exist(c.opFilename, 'file') == 2, 'Operational space properties file does not exist.');
%                 XmlOperations.XmlReadRemoveIndents(c.opFilename);
%             end

            % Checks for CUSTOM mode
            if CASPR_configuration.LoadGlobalModelMode() == ModelModeType.CUSTOM
                customFolderName = [c.modelFolderPath,'C/'];
                customFileName   = [customFolderName,'custom_',c.robotName];
                assert(exist([customFileName,'.c'], 'file') == 2, 'Source file for custom model does not exist.');
                assert(exist([customFileName,'.h'], 'file') == 2, 'Header file for custom model does not exist.');
                % Compile
                original_path = pwd;
                cd(customFolderName);
                libname = ['custom_',c.robotName];
                if libisloaded(libname)
                    unloadlibrary(libname);    
                end
                mex([libname,'.c'])
                % Load library
                loadlibrary(libname);
                cd(original_path); 
            end
            
            % Loads the bodiesModel and cable set to be used for the trajectory loading
            bodies_xmlobj = c.getBodiesPropertiesXmlObj();
            c.defaultCableSetId = char(c.cablesXmlObj.getDocumentElement.getAttribute('default_cable_set'));
            cableset_xmlobj = c.getCableSetXmlObj(c.defaultCableSetId);
            sysModel = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);            
            if(c.bodiesXmlObj.getElementsByTagName('operational_spaces').getLength~=0)
                c.defaultOperationalSetId = char(c.bodiesXmlObj.getElementsByTagName('operational_spaces').item(0).getAttribute('default_operational_set'));
                operationalset_xmlobj = c.getOperationalSetXmlObj(c.defaultOperationalSetId);
                sysModel.loadOperationalXmlObj(operationalset_xmlobj);
            end    
            c.bodiesModel = sysModel.bodyModel;
            c.displayRange = XmlOperations.StringToVector(char(bodies_xmlobj.getAttribute('display_range')));
            c.viewAngle = XmlOperations.StringToVector(char(bodies_xmlobj.getAttribute('view_angle')))';
        end
                
        function [sysModel] = getModel(obj, cable_set_id, operational_space_id)
            bodies_xmlobj = obj.getBodiesPropertiesXmlObj();
            cableset_xmlobj = obj.getCableSetXmlObj(cable_set_id);

            % Decide action according to the global_model_mode 
            model_mode = CASPR_configuration.LoadGlobalModelMode();
            if model_mode == ModelModeType.COMPILED
                % Check the reuse_compiled flag in
                % CASPR_environment variables
                is_reuse = false;                                
                reuse_compiled = CASPR_configuration.LoadReuseCompiled();
                % If the flag is turned on, warn the user
                if reuse_compiled
                    warning('off','all');  
                    CASPR_log.Warn('Flag: reuse_compiled is turned on.');
                    CASPR_log.Warn('Are you sure you want to reuse compiled files? Y/N [Y]');
                    str = input('', 's');
                    if str == 'y' || str == 'Y'
                        is_reuse = true;
                    else
                        % Turn the flag off if the user
                        % chose 'no'
                        CASPR_configuration.SetReuseCompiled(0);
                    end
                    warning('on','all');  
                end       
                % If the flag is on, also the user chose
                % 'y', then reuse previously compiled
                % files.
                if is_reuse                                        
                    sysModel = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj,ModelModeType.DEFAULT);   
                    % Operational
                    if nargin > 2
                        operationalset_xmlobj = obj.getOperationalSetXmlObj(operational_space_id);
                        sysModel.loadOperationalXmlObj(operationalset_xmlobj);                                                             
                        sysModel.bodyModel.updateOperationalSpace();  
                    end
                else
                    % Start Compilations...
                    warning('off','all');  
                    CASPR_log.Warn('Current version of compiled mode does not support changes in active <-> passive cables.');
                    CASPR_log.Warn('Compilation needs a long time. Please wait...');                        
                    warning('on','all');  
                    CASPR_log.Info('Preparation work...');
                    sysModel = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj,ModelModeType.COMPILED);
                    % Operational
                    if nargin > 2
                        operationalset_xmlobj = obj.getOperationalSetXmlObj(obj.defaultOperationalSetId);
                        sysModel.loadOperationalXmlObj(operationalset_xmlobj);
                        sysModel.bodyModel.updateOperationalSpace();
                    end
                    sysModel.compile();                                    
                end     
                sysModel.setFilesCompiled(true); 
                sysModel.setModelMode(ModelModeType.COMPILED);
            elseif model_mode == ModelModeType.CUSTOM
                % Use this two step approach for now
                % If op space can also be updated in custom mode,
                % then we can run CUSTOM mode at the first run                
                
                % Run DEFAULT mode first
                sysModel = SystemModel.LoadXmlObj(bodies_xmlobj,cableset_xmlobj,ModelModeType.DEFAULT);
                % Operational
                if nargin > 2
                    operationalset_xmlobj = obj.getOperationalSetXmlObj(operational_space_id);
                    sysModel.loadOperationalXmlObj(operationalset_xmlobj);
                    sysModel.bodyModel.updateOperationalSpace();
                end
                
                % Enter custom mode                
                sysModel.setModelMode(ModelModeType.CUSTOM);            
            else
                % DEFAULT || SYMBOLIC 
                sysModel = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj,model_mode);
                % Operational
                if nargin > 2
                    operationalset_xmlobj = obj.getOperationalSetXmlObj(operational_space_id);
                    sysModel.loadOperationalXmlObj(operationalset_xmlobj);
                    sysModel.bodyModel.updateOperationalSpace();
                end
            end
            sysModel.setRobotName(obj.robotName);
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
            trajectories_str = GUIOperations.XmlObj2StringCellArray(obj.trajectoriesXmlObj.getElementsByTagName('joint_trajectories').item(0).getChildNodes,'id');
        end
        
        function trajectories_str = getOperationalTrajectoriesList(obj)
            if (~isempty(obj.trajectoriesXmlObj.getElementsByTagName('operational_trajectories').item(0)))
                trajectories_str = GUIOperations.XmlObj2StringCellArray(obj.trajectoriesXmlObj.getElementsByTagName('operational_trajectories').item(0).getChildNodes,'id');
            else
                trajectories_str = {};
            end
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
end