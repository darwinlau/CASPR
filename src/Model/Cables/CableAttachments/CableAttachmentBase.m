% Abstract base class for CableAttachment
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%   Base class for the kinematics of an attachment point for a cable.
classdef (Abstract) CableAttachmentBase < handle
    properties (SetAccess = protected)
        link_num        % Link number that the attachment is attached to
        r_GA            % Relative attachment location ^kr_{GA} from COG to attachment location in local frame
        r_PA            % Relative attachment location ^kr_{PA} from joint to attachment locations in local frame
        
        r_OA            % Absolute attachment locations ^0r_{OA} in INERTIAL frame
    end
    
    methods 
        function update(obj, bodyKinematics)
            if (obj.link_num == 0)
                obj.r_OA = obj.r_PA;
            else
                obj.r_OA = bodyKinematics.bodies{obj.link_num}.R_0k * (bodyKinematics.bodies{obj.link_num}.r_OG + obj.r_GA);
            end
        end
    end
        
    methods (Abstract, Static)
        LoadXmlObj(attachment_xmlobj);
    end
end
