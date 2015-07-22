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
            switch type
                case ModelConfigType.M_PLANAR_XY
                    c.bodyPropertiesFilename = [c.root_folder, '\planar_xy\planar_xy_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\planar_xy\planar_xy_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\planar_xy\planar_xy_trajectories.xml'];
                case ModelConfigType.M_NECK_8S
                    c.bodyPropertiesFilename = [c.root_folder, '\8S_neck\8S_neck_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\8S_neck\8S_neck_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\8S_neck\8S_neck_trajectories.xml'];
                case ModelConfigType.M_2R_PLANAR_XZ
                    c.bodyPropertiesFilename = [c.root_folder, '\2R_planar_xz\2R_planar_xz_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\2R_planar_xz\2R_planar_xz_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\2R_planar_xz\2R_planar_xz_trajectories.xml'];
                case ModelConfigType.M_SPHERICAL_JOINT
                    c.bodyPropertiesFilename = [c.root_folder, '\spherical_joint\spherical_joint_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\spherical_joint\spherical_joint_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\spherical_joint\spherical_joint_trajectories.xml'];
                otherwise
                    error('ModelConfig type is not defined');
            end
            
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

