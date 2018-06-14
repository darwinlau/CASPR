% Represents the kinematics and dynamics of a single cable segment
% 
% Author        : Darwin LAU
% Created       : 2011
% Description	:
%   Cable segments are fundamental for MCDMs since cables can pass through
%   multiple links and hence have multiple segments.
classdef CableSegmentModel < handle   
    properties (SetAccess = private)
        numLinks        = -1            % The number of links
        attachments     = {}            % CEll array of two attachment locations (index 1 is the "from", index 2 is the "to")
        attached_links  = []            % Vector of link num that the two attachements
        
    end
        
    properties (Dependent)
        segmentVector                   % vector \mathbf{l}_{ij} in frame {0}
        length                          % length of segment
        length_offset                   % offset length that is in addition to the point to point segment depending on the attachment type (typically 0)
        CRM                             % Cable Routing Matrix for this cable and segment
    end
        
    methods
        function ck = CableSegmentModel(attachment_s, attachment_e, numLinks)
            ck.numLinks = numLinks;            
            ck.attachments{1} = attachment_s;
            ck.attachments{2} = attachment_e;
            ck.attached_links(1) = attachment_s.link_num;
            ck.attached_links(2) = attachment_e.link_num;            
        end
        
        function value = get.CRM(obj)
            value = zeros(1, obj.numLinks+1);
            if obj.attached_links(1) ~= obj.attached_links(2)
                value(obj.attached_links(1)+1) = -1;
                value(obj.attached_links(2)+1) = 1;  
            end
        end
        
        function c_k = getCRMTerm(obj, k)
            CASPR_log.Assert(k(length(k)) <= obj.numLinks+1, 'Invalid link number.');
            c_k = obj.CRM(k);
        end
        
        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done
        function clear(obj)
            obj.CRM = zeros(1, obj.numLinks+1);
            obj.attachments = {};
        end
        
        % The vectors r_OA are all in base frame
        function value = get.segmentVector(obj)
            l_segment = obj.attachments{2}.r_OA - obj.attachments{1}.r_OA;
            CASPR_log.Assert(l_segment ~= zeros(3,1), 'Cable segment vector should never be a zero vector');
            value = l_segment; 
        end
        
        function value = get.length(obj)
            value = sqrt(sum(obj.segmentVector.^2));
        end
        
        function value = get.length_offset(obj)
            % Only get the offset from the first attachment for now, unless
            % in the future the last segment also needs the second
            % attachment length_offset
            value = obj.attachments{1}.length_offset;
        end
    end
    
end
