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
        type                        % Type of model from ModelConfigType enum
        
        bodyPropertiesFilename      % Filename for the body properties
        cablesPropertiesFilename    % Filename for the cable properties
        trajectoriesFilename        % Filename for the trajectories
        opFilename                  % Filename for the operational space
        
        bodiesModel                 % Stores the SystemModelBodies object for the robot model
        displayRange                % The axis range to display the robot
        viewAngle                   % The angle for which the model should be viewed at
        defaultCableSetId           % ID for the default cable set to display first
    end
    
    properties (Access = private)
        root_folder                 % The root folder for the CASPR build
        
        bodiesXmlObj                % The DOMNode object for body props
        cablesXmlObj                % The DOMNode object for cable props
        trajectoriesXmlObj          % The DOMNode for trajectory props
        opXmlObj                    % The DOMNode for operational space
    end
    
    methods
        % Constructor for the ModelConfig class. This builds the xml
        % objects.
        function c = ModelConfigBase(type, folder, list_file)
            c.type = type;
            c.root_folder = [fileparts(mfilename('fullpath')), folder];
            c.opFilename = '';
            c.opXmlObj = [];
            
            % Check the type of enum and open the master list
            fid = fopen([c.root_folder, list_file]);
            
            % Determine the Filenames
            % Load the contents
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');            
            type_string = char(type);
            i_length = length(cell_array{1});
            status_flag = 1;
            % Loop through until the right line of the list is found
            for i = 1:i_length
                if(strcmp(char(cell_array{1}{i}),type_string))
                    cdpr_folder                 = char(cell_array{2}{i});
                    c.bodyPropertiesFilename    = [c.root_folder, cdpr_folder,char(cell_array{3}{i})];
                    c.cablesPropertiesFilename  = [c.root_folder, cdpr_folder,char(cell_array{4}{i})];
                    c.trajectoriesFilename      = [c.root_folder, cdpr_folder,char(cell_array{5}{i})];
                    if(~isempty(cell_array{6}{i}))
                        c.opFilename = [c.root_folder, cdpr_folder,char(cell_array{6}{i})];
                    end
                    fclose(fid);
                    status_flag = 0;
                    break;
                end
            end
            if(status_flag)
                error('ModelConfig type is not defined');
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
            if (~isempty(c.opFilename))
                assert(exist(c.opFilename, 'file') == 2, 'Operational space properties file does not exist.');
                XmlOperations.XmlReadRemoveIndents(c.opFilename);
            end
            
            % Read the model config related properties from the bodies and
            % cables XML
            c.defaultCableSetId = char(c.cablesXmlObj.getDocumentElement.getAttribute('default_cable_set'));
            c.displayRange = XmlOperations.StringToVector(char(c.bodiesXmlObj.getDocumentElement.getAttribute('display_range')));
            c.viewAngle = XmlOperations.StringToVector(char(c.bodiesXmlObj.getDocumentElement.getAttribute('view_angle')));
            % Loads the bodiesModel to be used for the trajectory loading
            bodies_xmlobj = c.getBodiesPropertiesXmlObj();
            cableset_xmlobj = c.getCableSetXmlObj(c.defaultCableSetId);
            sysModel = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);
            c.bodiesModel = sysModel.bodyModel;
        end
                
        function [sysModel] = getModel(obj, cable_set_id, operational_space_id)
            bodies_xmlobj = obj.getBodiesPropertiesXmlObj();
            cableset_xmlobj = obj.getCableSetXmlObj(cable_set_id);
            sysModel = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);
            
            if (nargin >= 3 && ~isempty(operational_space_id))
                op_xmlobj = obj.getOPXmlObj(op_set_id);
                sysModel.loadOpXmlObj(op_xmlobj);
            end
        end
        
        function [traj] = getTrajectory(obj, trajectory_id)
            traj_xmlobj = obj.getTrajectoryXmlObj(trajectory_id);
            traj = JointTrajectory.LoadXmlObj(traj_xmlobj, obj.bodiesModel);
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
        
        function trajectories_str = getTrajectoriesList(obj)
            trajectories_str = GUIOperations.XmlObj2StringCellArray(obj.trajectoriesXmlObj.getElementsByTagName('trajectories').item(0).getElementsByTagName('trajectory'),'id');
        end
    end
    
    methods (Access = private)
        % Gets the body properties xml object
        function v = getBodiesPropertiesXmlObj(obj)
            v = obj.bodiesXmlObj.getDocumentElement;
        end
        
        % Gets the cable set properties xml object
        function v = getCableSetXmlObj(obj, id)
            v = obj.cablesXmlObj.getElementById(id);
            assert(~isempty(v), sprintf('Id ''%s'' does not exist in the cables XML file', id));
        end
        
        % Get the trajectory xml object
        function v = getTrajectoryXmlObj(obj, id)
            v = obj.trajectoriesXmlObj.getElementById(id);
            assert(~isempty(v), sprintf('Id ''%s'' does not exist in the trajectories XML file', id));
        end
        
        % Get the operational space xml object
        function v = getOPXmlObj(obj, id)
            v = obj.opXmlObj.getElementById(id);
            assert(~isempty(v), sprintf('Id ''%s'' does not exist in the operation XML file', id));
        end
    end
end

