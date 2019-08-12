% Abstract base class for CableAttachment
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%   Base class for the kinematics of an attachment point for a cable.
classdef (Abstract) CableAttachmentBase < handle
    properties (SetAccess = protected)
        link_num            % Link number that the attachment is attached to
        r_PG                % Relative vector ^kr_{PG} from joint location to COG in local frame
        r_GA                % Relative attachment location ^kr_{GA} from COG to attachment location in local frame
        r_OA                % Absolute attachment locations ^0r_{OA} in INERTIAL frame        
        
        length_offset = 0   % Length offset that depends on the type of cable attachment (particularly for moving attachment points). Typically this is 0.
    end
    
    methods 
        function update(obj, bodyKinematics)
            if (obj.link_num == 0)
                obj.r_OA = obj.r_GA;
            else
                obj.r_OA = bodyKinematics.bodies{obj.link_num}.R_0k * (bodyKinematics.bodies{obj.link_num}.r_OG + obj.r_GA);
            end
        end
        
        function directUpdate(obj, r_OA)
            obj.r_OA = r_OA;
        end
    end
    
    methods (Abstract)
        updateAttachmentLocation(obj, r_A, attachment_ref_type);
    end
        
    methods (Abstract, Static)
        LoadXmlObj(attachment_xmlobj);
    end
end
