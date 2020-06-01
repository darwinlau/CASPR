% This type of attachment is with a kingpin which rotates the cable outlet
% according to the direction of next attachment. For details please refers
% to explaination on https://github.com/darwinlau/CASPR/issues/4
%
% Author        : Darwin LAU
% Created       : 2016
% Description	:
classdef BaseRotatingPulleyAttachment < CableAttachmentBase
    properties (SetAccess = private)
        constantPulleyPoint        
        pulleyCentrePoint        
        nextAttachment
    end
    
    properties (Access = private)
        pulley_radius
        cable_entry_vector
    end
    
    methods
        function ca = BaseRotatingPulleyAttachment(pulley_point, pulley_radius, cable_entry_vector, next_attachment)
            ca.link_num = 0;
            ca.constantPulleyPoint = pulley_point;
            ca.nextAttachment = next_attachment;
            ca.pulley_radius = pulley_radius;
            ca.cable_entry_vector = cable_entry_vector;
            ca.r_GA = [0;0;0];
        end
        
        function update(obj, bodyKinematics)
            obj.nextAttachment.update(bodyKinematics);
            [obj.r_GA, obj.length_offset, obj.pulleyCentrePoint] = obj.ComputePulleyLeavingLocation(obj.constantPulleyPoint, obj.cable_entry_vector, obj.nextAttachment.r_OA, obj.pulley_radius);
            
            update@CableAttachmentBase(obj, bodyKinematics);
        end
        
        function updateAttachmentLocation(obj, r_A, ~)
            CASPR_log.Error('updateAttachmentLocation function not implemented for BaseRotatingPulleyAttachment yet');
            % Re-look at whether this should be the implementation
            obj.constantPulleyPoint = r_A;
        end
    end
    
    methods (Static)
        function ca = LoadXmlObj(xmlObj, next_attachment)
            pulley_point = XmlOperations.StringToVector3(char(xmlObj.getElementsByTagName('fixed_location').item(0).getFirstChild.getData));
            pulley_radius = str2double(xmlObj.getElementsByTagName('pulley_radius').item(0).getFirstChild.getData);
            cable_entry_vector = XmlOperations.StringToVector3(char(xmlObj.getElementsByTagName('cable_entry_vector').item(0).getFirstChild.getData));
            
            ca = BaseRotatingPulleyAttachment(pulley_point, pulley_radius, cable_entry_vector, next_attachment);  
        end
        
        % Function to determine the leaving location of the pulley given:
        % INPUTS
        %   - r_F: Constant location on the pulley where cable enters
        %   - v_F: Direction vector of the cable entering r_F
        %   - r_B: Location of the other end of the cable on the robot
        %   - R: Radius of the pulley
        % OUTPUTS
        %   - r_A: The position of the cable leaving the pulley
        %   - l_arc: length of the arc on the pulley
        %   - r_C: The position of the centre of the pulley location
        function [r_A, l_arc, r_C] = ComputePulleyLeavingLocation(r_F, v_F, r_B, R)            
%             n_plane = cross(v_A, r_B - r_A);
%             p_plane = [0;0;0];
%             n_c_perpendicular = v_A;
%             p_c_perpendicular = [0;0;0];
%             
%             [p_C, v_C, ~] = PlaneOperations.PlaneIntersection(n_c_perpendicular, p_c_perpendicular, n_plane, p_plane);
%             a = v_C' * v_C;
%             b = 2 * p_C' * v_C;
%             c = p_C' * p_C - R^2;
%             t = (-b - sqrt(b^2 - 4 * a * c))/(2*a);
%             r_C = r_A + (p_C + t*v_C);
%             
%             n_d_perpendicular = (r_C - r_B);
%             p_d_perpendicular = [0; 0; -R^2/n_d_perpendicular(3)];
%             
%             [p_D, v_D, ~] = PlaneOperations.PlaneIntersection(n_d_perpendicular, p_d_perpendicular, n_plane, p_plane);
%             a = v_D' * v_D;
%             b = 2 * p_D' * v_D;
%             c = p_D' * p_D - R^2;
%             t = (-b - sqrt(b^2 - 4 * a * c))/(2*a);
%             r_D = r_C + (p_D + t*v_D)
%             
%             n1 = (r_B - r_D)/norm(r_B-r_D);
%             n2 = v_A/norm(v_A);
%             l_arc = R * acos(n1' * n2);
            
            % First calculate the position of the centre of pulley
            % This must be in the plane of the pulley and also
            % perpendicular to the rotating axis
            v_FB = r_B - r_F;
            n_plane = cross(v_F, v_FB);
            n_plane_unit = n_plane/norm(n_plane);
            v_FC = cross(n_plane, v_F);
            r_C = r_F + R*v_FC/norm(v_FC);
            
            % Next calculate the position of the point leaving pulley
            rho = R/norm(r_B - r_C);
            
            r_A = r_C + rho^2 * (r_B - r_C) - rho * sqrt(1 - rho^2) * cross(n_plane_unit, r_B - r_C);
            
            % Finally calculate the arc length (cable length)
            n1 = (r_F - r_C)/norm(r_F - r_C);
            n2 = (r_A - r_C)/norm(r_A - r_C);
            l_arc = R * acos(n1' * n2);
        end
    end
end
