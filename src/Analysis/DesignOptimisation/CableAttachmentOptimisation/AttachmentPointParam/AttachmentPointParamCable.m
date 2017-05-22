% Representation of a set of attachment points for a single cable. For
% single link CDPRs, it would only contain two attachments, but may be more
% for MCDMs. 
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	: 
%   Currently, the representation makes a few assumptions: 1) Cable routing
%   is not considered; 2) it is assumed that cables are attached to all 
%   links; 3) cables are attached in order from base to furthest link; 4)
%   only one attachment per link is allowed.
classdef AttachmentPointParamCable < handle    
    
    properties (SetAccess = private)
        attachments         % Cell array of AttachmentPointParamBase
    end
    
    properties (Dependent)
        numVars             % Total number of attachment parameters
        x                   % The cable attachment state representation x
        x_min               % Min values for x
        x_max               % Max values for x
    end
    
    methods
        function ap = AttachmentPointParamCable(attachs)
            ap.attachments = attachs;
        end
        
        % Function responsible for updating the individual state variables
        % of each attachment point
        function updateCableAttachments(obj, x) %, cableKin, bodiesKin) % Maybe bodiesKin can be used later for updating 
            counter = 0;
            for k = 1:length(obj.attachments)
                xk = x(counter+1:counter+obj.attachments{k}.numVars);
                counter = counter + obj.attachments{k}.numVars;
                obj.attachments{k}.update(xk);
            end
            
            
%             % This is temporary and assumes that the attachments are from
%             % base to end-effector link by link
%             for k = 1:cableKin.numLinks
%                 attachment_s = CableAttachmentPoint(k-1, obj.attachments{k}.r_a, obj.attachments{k}.attachmentRefType, bodiesKin);
%                 attachment_e = CableAttachmentPoint(k, obj.attachments{k+1}.r_a, obj.attachments{k+1}.attachmentRefType, bodiesKin);
%                 segment = CableSegmentModel(attachment_s, attachment_e, cableKin.numLinks);
%                 cableKin.addSegment(segment);
%             end
        end
        
        function value = get.numVars(obj)
            value = 0;
            for i = 1:length(obj.attachments)
                value = value + obj.attachments{i}.numVars;
            end
        end
        
        function value = get.x(obj)
            counter = 0;
            value = zeros(obj.numVars, 1);
            for i = 1:length(obj.attachments)
                value(counter+1:counter+obj.attachments{i}.numVars) = obj.attachments{i}.x;
                counter = counter + obj.attachments{i}.numVars;
            end
        end
        
        function value = get.x_min(obj)
            counter = 0;
            value = zeros(obj.numVars, 1);
            for i = 1:length(obj.attachments)
                value(counter+1:counter+obj.attachments{i}.numVars) = obj.attachments{i}.x_min;
                counter = counter + obj.attachments{i}.numVars;
            end
        end
        
        function value = get.x_max(obj)
            counter = 0;
            value = zeros(obj.numVars, 1);
            for i = 1:length(obj.attachments)
                value(counter+1:counter+obj.attachments{i}.numVars) = obj.attachments{i}.x_max;
                counter = counter + obj.attachments{i}.numVars;
            end
        end
    end
end

