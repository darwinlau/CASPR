% Class to store the configuration of different robots from the XML files
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :
%    This class stores the locations for all of the different CDPRs that
%    are accessible within CASPR. The CDPR information is stored as XML
%    files. New robots that are added must also be added to the ModelConfig
%    in order for it to be accessible.
classdef ModelConfig   
    properties (SetAccess = private)
        type                        % Type of joint from JointType enum
        bodyPropertiesFilename      % Filename for the body properties
        cablesPropertiesFilename    % Filename for the cable properties
        trajectoriesFilename        % Filename for the trajectories
        opFilename                  % Filename for the operational space
        
        bodiesXmlObj                % The DOMNode object for body props
        cablesXmlObj                % The DOMNode object for cable props
        trajectoriesXmlObj          % The DOMNode for trajectory props
        opXmlObj                    % The DOMNode for operational space
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
                    cdpr_folder = '\SCDM\planar_manipulators\simple_planar_xy\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'simple_planar_xy_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'simple_planar_xy_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'simple_planar_xy_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'simple_planar_xy_op.xml'];
                case ModelConfigType.M_2DOF_VSD
                    cdpr_folder = '\SCDM\planar_manipulators\2_Dof_VSD\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, '2_Dof_VSD_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, '2_Dof_VSD_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, '2_Dof_VSD_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, '2_Dof_VSD_op.xml'];
                case ModelConfigType.M_SIMPLE_SPHERICAL
                    cdpr_folder = '\SCDM\spherical_manipulators\simple_spherical\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'simple_spherical_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'simple_spherical_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'simple_spherical_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'simple_spherical_op.xml'];
                case ModelConfigType.M_SIMPLE_SPATIAL
                    cdpr_folder = '\SCDM\spatial_manipulators\simple_spatial\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'simple_spatial_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'simple_spatial_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'simple_spatial_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'simple_spatial_op.xml'];
                % SCDMs from different laboratories and hardware
                case ModelConfigType.M_MYOROB_SHOULDER
                    cdpr_folder = '\SCDM\spherical_manipulators\myorob_shoulder_TUM\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'myorob_shoulder_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'myorob_shoulder_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'myorob_shoulder_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'myorob_shoulder_op.xml'];
                case ModelConfigType.M_ACROBOT
                    cdpr_folder = '\SCDM\spatial_manipulators\ACROBOT_IRTJV\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'ACROBOT_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'ACROBOT_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'ACROBOT_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'ACROBOT_op.xml'];
                case ModelConfigType.M_IPANEMA_2
                    cdpr_folder = '\SCDM\spatial_manipulators\IPAnema2_FraunhoferIPA\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'IPAnema2_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'IPAnema2_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'IPAnema2_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'IPAnema2_op.xml'];
                case ModelConfigType.M_NIST_ROBOCRANE
                    cdpr_folder = '\SCDM\spatial_manipulators\NIST_ROBOCRANE\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'NIST_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'NIST_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'NIST_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'NIST_op.xml'];
				case ModelConfigType.M_COGIRO
                    cdpr_folder = '\SCDM\spatial_manipulators\CoGiRo\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'CoGiRo_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'CoGiRo_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'CoGiRo_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'CoGiRo_op.xml'];
                case ModelConfigType.M_KNTU_PLANAR
                    cdpr_folder = '\SCDM\planar_manipulators\KNTU_planar\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'KNTU_planar_xy_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'KNTU_planar_xy_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'KNTU_planar_xy_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'KNTU_planar_xy_op.xml'];
                % Multi-link Cable Driven Manipulators (MCDM)
                case ModelConfigType.M_NECK_8S
                    cdpr_folder = '\MCDM\8S_neck\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, '8S_neck_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, '8S_neck_cables.xml'];
                    %c.trajectoriesFilename = [c.root_folder, cdpr_folder, '8S_neck_trajectories.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, '8S_neck_trajectories_eulerXYZ.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, '8S_neck_op.xml'];
                case ModelConfigType.M_2R_PLANAR_XZ
                    cdpr_folder = '\MCDM\2R_planar_xz\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, '2R_planar_xz_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, '2R_planar_xz_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, '2R_planar_xz_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, '2R_planar_xz_op.xml'];
                case ModelConfigType.M_CAREX
                    cdpr_folder = '\MCDM\CAREX\';
                    c.bodyPropertiesFilename = [c.root_folder, cdpr_folder, 'CAREX_bodies.xml'];
                    c.cablesPropertiesFilename = [c.root_folder, cdpr_folder, 'CAREX_cables.xml'];
                    c.trajectoriesFilename = [c.root_folder, cdpr_folder, 'CAREX_trajectories.xml'];
                    c.opFilename = [c.root_folder, cdpr_folder, 'CAREX_op.xml'];
                otherwise
                    error('ModelConfig type is not defined');
            end
            
            if(isunix)
                c.bodyPropertiesFilename = strrep(c.bodyPropertiesFilename, '\', '/');
                c.cablesPropertiesFilename = strrep(c.cablesPropertiesFilename, '\', '/');
                c.trajectoriesFilename = strrep(c.trajectoriesFilename, '\', '/');
                c.opFilename = strrep(c.opFilename, '\', '/');
            end
            
            assert(exist(c.bodyPropertiesFilename, 'file') == 2, 'Body properties file does not exist.');
            assert(exist(c.cablesPropertiesFilename, 'file') == 2, 'Cable properties file does not exist.');
            assert(exist(c.trajectoriesFilename, 'file') == 2, 'Trajectories file does not exist.');
            assert(exist(c.opFilename, 'file') == 2, 'Operational space coordinates file does not exist.');
            
            c.bodiesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.bodyPropertiesFilename);
            c.cablesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.cablesPropertiesFilename);
            c.trajectoriesXmlObj =  XmlOperations.XmlReadRemoveIndents(c.trajectoriesFilename);
            c.opXmlObj  =    XmlOperations.XmlReadRemoveIndents(c.opFilename);
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
        
        function v = getOPXmlObj(obj, id)
            v = obj.opXmlObj.getElementById(id);
            assert(~isempty(v), sprintf('Id ''%s'' does not exist in the operation XML file', id));
        end
    end
end

