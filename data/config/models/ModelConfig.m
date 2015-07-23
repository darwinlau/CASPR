classdef ModelConfig
    %JOINT Summary of this class goes here
    %   Detailed explanation goes here
   
    properties (SetAccess = private)
        type                % Type of joint from JointType enum
        bodyPropertiesFilename
        cablesPropertiesFilename
        trajectoriesFilename
        
        bodiesXmlObj
        cablesXmlObj
        trajectoriesXmlObj
    end
    
    properties (Access = private)
        root_folder
    end
    
    methods
        function c = ModelConfig(type)
            c.type = type;
            c.root_folder = fileparts(mfilename('fullpath'));
            if(isunix)
                dlm = '/';
            else
                dlm = '\';
            end
            switch type
                case ModelConfigType.M_PLANAR_XY
                    model_folder                =   'planar_xy';
                case ModelConfigType.M_NECK_8S
                    model_folder                =   '8S_neck';
                case ModelConfigType.M_2R_PLANAR_XZ
                    model_folder                =   '2R_planar_xz';
                case ModelConfigType.M_SPHERICAL_JOINT
                    model_folder                =   'spherical_joint';
                otherwise
                    error('ModelConfig type is not defined');
            end
            c.bodyPropertiesFilename    = [c.root_folder,dlm,model_folder,dlm,model_folder,'_bodies.xml'];
            c.cablesPropertiesFilename  = [c.root_folder,dlm,model_folder,dlm,model_folder,'_cables.xml'];
            c.trajectoriesFilename      = [c.root_folder,dlm,model_folder,dlm,model_folder,'_trajectories.xml'];
            c.bodiesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.bodyPropertiesFilename);
            c.cablesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.cablesPropertiesFilename);
            c.trajectoriesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.trajectoriesFilename);
        end
        
        function v = getBodiesProperiesXmlObj(obj)
            v = obj.bodiesXmlObj.getDocumentElement;
        end
        
        function v = getCableSetXmlObj(obj, id)
            v = obj.cablesXmlObj.getElementById(id);
            assert(~isempty(v), sprintf('Id ''%s'' does not exist in the cables XML file', id));
        end
        
        function v = getTrajectoryXmlObj(obj, id)
            v = obj.trajectoriesXmlObj.getElementById(id);
            assert(~isempty(v), sprintf('Id ''%s'' does not exist in the trajectories XML file', id));
        end
    end
end

