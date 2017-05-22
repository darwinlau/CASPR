% A parameterised representation of cable attachment locations 
% where cables are attachable in a cartesian (XYZ) space
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%   It is currently assumed that the centre of the cartesian frame is at
%   (0,0,0). In the future a centre point and orientation should be added
%   to this.

classdef AttachmentPointParamCartesian < AttachmentPointParamBase    
    properties (Constant)
        numVars = 3;
    end
    
    properties (SetAccess = private)
        x_range;
        y_range;
        z_range;
    end
    
    properties (Dependent)
        x_min
        x_max
    end
    
    methods
        function ap = AttachmentPointParamCartesian(attachment, attachmentRefType, x_range, y_range, z_range)
            ap@AttachmentPointParamBase(attachment, attachmentRefType);
            ap.x_range = x_range;
            ap.y_range = y_range;
            ap.z_range = z_range;
        end
        
        function value = get.x_min(obj)
            value = [obj.x_range(1); obj.y_range(1); obj.z_range(1)];
        end
        
        function value = get.x_max(obj)
            value = [obj.x_range(2); obj.y_range(2); obj.z_range(2)];
        end
    end
    
    methods (Access = protected)
        function r = paramToAttachments(~, x)
            r = x;
        end
    end
end

