classdef BodyDynamicsRigid < BodyDynamics
    %BODYSYSTEMKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here

    methods
        function bd = BodyDynamicsRigid(id)
            bd@BodyDynamics(id);
        end

        function update(obj, bodyKinematics)
            % Pre update

            % Update standard things from kinematics
            update@BodyDynamics(obj, bodyKinematics);

            % Post updates if necessary
        end
    end

    methods (Static)
        function [bk] = LoadXmlObj(xmlobj)
            id = str2double(char(xmlobj.getAttribute('num')));
            physicalObj = xmlobj.getElementsByTagName('physical').item(0);
            inertiaObj = physicalObj.getElementsByTagName('inertia').item(0);

            I_Gxx = str2double(inertiaObj.getElementsByTagName('Ixx').item(0).getFirstChild.getData);
            I_Gyy = str2double(inertiaObj.getElementsByTagName('Iyy').item(0).getFirstChild.getData);
            I_Gzz = str2double(inertiaObj.getElementsByTagName('Izz').item(0).getFirstChild.getData);
            I_Gxy = str2double(inertiaObj.getElementsByTagName('Ixy').item(0).getFirstChild.getData);
            I_Gxz = str2double(inertiaObj.getElementsByTagName('Ixz').item(0).getFirstChild.getData);
            I_Gyz = str2double(inertiaObj.getElementsByTagName('Iyz').item(0).getFirstChild.getData);

            bk = BodyDynamicsRigid(id);
            bk.m = str2double(physicalObj.getElementsByTagName('mass').item(0).getFirstChild.getData);
            bk.I_G = [I_Gxx I_Gxy I_Gxz; ...
                I_Gxy I_Gyy I_Gyz; ...
                I_Gxz I_Gyz I_Gzz];
        end
    end

end