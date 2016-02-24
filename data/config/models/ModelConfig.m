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
                % Single-link Cable Driven Manipulators (SCDM)
                % Standard simple SCDMs for reference
                case ModelConfigType.M_SIMPLE_PLANAR_XY
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\planar_manipulators\simple_planar_xy\simple_planar_xy_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\planar_manipulators\simple_planar_xy\simple_planar_xy_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\planar_manipulators\simple_planar_xy\simple_planar_xy_trajectories.xml'];
                case ModelConfigType.M_SIMPLE_SPHERICAL
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\spherical_manipulators\simple_spherical\simple_spherical_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\spherical_manipulators\simple_spherical\simple_spherical_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\spherical_manipulators\simple_spherical\simple_spherical_trajectories.xml'];
                case ModelConfigType.M_SIMPLE_SPATIAL
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\simple_spatial\simple_spatial_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\simple_spatial\simple_spatial_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\spatial_manipulators\simple_spatial\simple_spatial_trajectories.xml'];
                % SCDMs from different laboratories and hardware
                case ModelConfigType.M_MYOROB_SHOULDER
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\spherical_manipulators\myorob_shoulder_TUM\myorob_shoulder_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\spherical_manipulators\myorob_shoulder_TUM\myorob_shoulder_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\spherical_manipulators\myorob_shoulder_TUM\myorob_shoulder_trajectories.xml'];
                case ModelConfigType.M_ACROBOT
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\ACROBOT_IRTJV\ACROBOT_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\ACROBOT_IRTJV\ACROBOT_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\spatial_manipulators\ACROBOT_IRTJV\ACROBOT_trajectories.xml'];
                case ModelConfigType.M_IPANEMA_2
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\IPAnema2_FraunhoferIPA\IPAnema2_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\IPAnema2_FraunhoferIPA\IPAnema2_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\spatial_manipulators\IPAnema2_FraunhoferIPA\IPAnema2_trajectories.xml'];
                case ModelConfigType.M_NIST_ROBOCRANE
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\NIST_ROBOCRANE\NIST_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\NIST_ROBOCRANE\NIST_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\spatial_manipulators\NIST_ROBOCRANE\NIST_trajectories.xml'];
				case ModelConfigType.M_COGIRO
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\CoGiRo\CoGiRo_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\spatial_manipulators\CoGiRo\CoGiRo_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\spatial_manipulators\CoGiRo\CoGiRo_trajectories.xml'];
                case ModelConfigType.M_KNTU_PLANAR
                    c.bodyPropertiesFilename = [c.root_folder, '\SCDM\planar_manipulators\KNTU_planar\KNTU_planar_xy_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\SCDM\planar_manipulators\KNTU_planar\KNTU_planar_xy_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\SCDM\planar_manipulators\KNTU_planar\KNTU_planar_xy_trajectories.xml'];
                % Multi-link Cable Driven Manipulators (MCDM)
                case ModelConfigType.M_NECK_8S
                    c.bodyPropertiesFilename = [c.root_folder, '\MCDM\8S_neck\8S_neck_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\MCDM\8S_neck\8S_neck_cables.xml'];
                    %c.trajectoriesFilename = [c.root_folder, '\MCDM\8S_neck\8S_neck_trajectories.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\MCDM\8S_neck\8S_neck_trajectories_eulerXYZ.xml'];
                case ModelConfigType.M_2R_PLANAR_XZ
                    c.bodyPropertiesFilename = [c.root_folder, '\MCDM\2R_planar_xz\2R_planar_xz_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\MCDM\2R_planar_xz\2R_planar_xz_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\MCDM\2R_planar_xz\2R_planar_xz_trajectories.xml'];
                case ModelConfigType.M_CAREX
                    c.bodyPropertiesFilename = [c.root_folder, '\MCDM\CAREX\CAREX_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, '\MCDM\CAREX\CAREX_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, '\MCDM\CAREX\CAREX_trajectories.xml'];
                otherwise
                    error('ModelConfig type is not defined');
            end
            
            if(isunix)
                c.bodyPropertiesFilename = strrep(c.bodyPropertiesFilename, '\', '/');
                c.cablesPropertiesFilename = strrep(c.cablesPropertiesFilename, '\', '/');
                c.trajectoriesFilename = strrep(c.trajectoriesFilename, '\', '/');
            end
            
            assert(exist(c.bodyPropertiesFilename, 'file') == 2, 'Body properties file does not exist.');
            assert(exist(c.cablesPropertiesFilename, 'file') == 2, 'Cable properties file does not exist.');
            assert(exist(c.trajectoriesFilename, 'file') == 2, 'Trajectories file does not exist.');
            
            c.bodiesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.bodyPropertiesFilename);
            c.cablesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.cablesPropertiesFilename);
            c.trajectoriesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.trajectoriesFilename);
        end
        
        function v = getBodiesPropertiesXmlObj(obj)
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

