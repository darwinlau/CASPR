classdef BodyKinematicsRigid < BodyKinematics
    %BODYSYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here

    methods
        function bk = BodyKinematicsRigid(id, name, jointType)
            bk@BodyKinematics(id, name, jointType);
        end

        function update(obj, q, q_dot, q_ddot)
            % Can update states (such as local attachments) first if necessary
            % Since bodies and attachment locations do not change, no need to update anything here

            % Update standard things from kinematics
            update@BodyKinematics(obj, q, q_dot, q_ddot);

            % Post updates if necessary
        end
    end

    methods (Static)
        function [bk] = LoadXmlObj(xmlobj)
            id = str2double(char(xmlobj.getAttribute('num')));
            name = char(xmlobj.getAttribute('name'));
            jointTypeObj = xmlobj.getElementsByTagName('joint_type').item(0);
            parentObj = xmlobj.getElementsByTagName('parent').item(0);
            physicalObj = xmlobj.getElementsByTagName('physical').item(0);

            bk = BodyKinematicsRigid(id, name, JointType.(char(jointTypeObj.getFirstChild.getData)));
            bk.r_G = XmlOperations.StringToVector3(char(physicalObj.getElementsByTagName('com_location').item(0).getFirstChild.getData));
            bk.r_Pe = XmlOperations.StringToVector3(char(physicalObj.getElementsByTagName('end_location').item(0).getFirstChild.getData));
<<<<<<< HEAD

=======
            
>>>>>>> e893c5821135d54bdc11523761849f2d1c95cb1b
            bk.r_Parent = XmlOperations.StringToVector3(char(parentObj.getElementsByTagName('location').item(0).getFirstChild.getData));
            bk.parentLinkId = str2double(char(parentObj.getElementsByTagName('num').item(0).getFirstChild.getData));
        end
    end

end
