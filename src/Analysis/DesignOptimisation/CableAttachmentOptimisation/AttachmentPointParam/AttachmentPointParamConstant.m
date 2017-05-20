% The simplest parameterised representation of cable attachment locations 
% by assuming the location is constant used for cable attachment 
% optimisation 
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:

classdef AttachmentPointParamConstant < AttachmentPointParamBase
    properties (Constant)
        numVars = 0;
    end
    
    properties (Dependent)
        x_min
        x_max
    end
        
    methods
        function ap = AttachmentPointParamConstant(attachment)
            ap@AttachmentPointParamBase(attachment, CableAttachmentReferenceType.JOINT);
            ap.x = [];
        end
        
        function value = get.x_min(~)
            value = [];
        end
        
        function value = get.x_max(~)
            value = [];
        end
    end
    
    methods 
        function update(~, ~)
        end
    end
    
    methods (Access = protected)
        function r_a = paramToAttachments(obj, ~)
            r_a = obj.attachment.r_GA;
        end
    end
end

