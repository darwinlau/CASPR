% Abstract class of the parameterised representation of cable attachment
% locations used for cable attachment optimisation
%
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%	Abstract data structre, children classes must implement the
%	determineAttachment function. Note that all attachment vectors are
%	expressed with respect to the local frame of the link.
classdef (Abstract) AttachmentPointParamBase < handle
    properties (Constant, Abstract)
        numVars             % Number of variables of the representation
    end

    properties (SetAccess = protected)
        x = []              % State of the attachment
        r_a = [0; 0; 0]     % Attachment vector (a_x, a_y, a_z)
    end

    properties (SetAccess = private)
        attachmentRefType   % CableAttachmentReferenceType (CoM or joint)
        cableAttachment     % CableAttachmentBase object (inherited)
    end

    properties (Abstract, Dependent)
        x_min
        x_max
    end

    methods
        function ap = AttachmentPointParamBase(attachment, attachmentRefType)
            ap.attachmentRefType = attachmentRefType;
            ap.cableAttachment = attachment;
            ap.x = zeros(ap.numVars, 1);
        end
    end

    methods (Abstract, Access = protected)
        % Conversion from x to r_a
        r = paramToAttachments(obj, x);
    end

    methods
        function update(obj, x)
            CASPR_log.Assert(all(size(x) == [obj.numVars 1]), sprintf('x must be a vector of length %d\n', obj.numVars));
            obj.x = x;
            obj.r_a = obj.paramToAttachments(x);
            % Just assume the attachments are all PointAttachment for now
            obj.cableAttachment.updateAttachmentLocation(obj.r_a, obj.attachmentRefType);
        end
    end

end
