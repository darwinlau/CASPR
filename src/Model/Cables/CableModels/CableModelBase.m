% Abstract class for the kinematics and dynamics of a single cable
% 
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102�1113,
% Oct. 2013.
% 
% Author        : Darwin LAU
% Created       : 2011
% Description	:
%	Data structure that contains the kinematics of a multisegment cable. It
%	provides:
%       - the ability to add and remove segments
%       - update the kinematics (i.e. attachment locations) of each segment
%       - access the CRM term
classdef (Abstract) CableModelBase < handle
    properties (Constant)
        INVALID_FORCE = -1
    end
    
    properties 
        % Actual cable force, use -1 to represent invalid forces
        force = CableModelBase.INVALID_FORCE;
    end
    
    properties (SetAccess = protected)
        % The segments objects will use the attachments objects
        segments = {}               % Cell array of CableSegmentModel objects
        attachments = {}            % Cell array of CableAttachment objects
        
        isActive = true             % Whether the cable is in passive state
        % Minimum and maximum allowable cable force
        forceMin
        forceMax
    end    
    
    properties (SetAccess = private)
        name        = '';                  % Cable name
        numLinks    = -1;
    end
    
    properties (Dependent)
        length                      % Length of the cable
        numSegments                 % Total number of segments
    end
    
    properties (Abstract, Dependent)
        K                           % Stiffness of the cable
    end
    
    methods (Abstract, Static)
        % Function to load the XML properties for this cable
        LoadXmlObj(xml, bodiesModel);
    end
        
    methods
        function ck = CableModelBase(name, numLinks)
            ck.name = name;
            ck.numLinks = numLinks;
        end
                
        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done
        function addSegment(obj, segment)
            obj.segments{obj.numSegments + 1} = segment;
        end
        
        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done
        function clear(obj)
            obj.segments = {};
            obj.attachments = {};
        end
        
        function update(obj, bodyKinematics)
            for a = 1:length(obj.attachments)
                obj.attachments{a}.update(bodyKinematics);
            end
        end
        
        function value = get.length(obj)
            value = 0;
            for j = 1:obj.numSegments
                value = value + obj.segments{j}.length;
            end
        end
        
        function value = get.numSegments(obj)
            value = length(obj.segments);
        end
        
        function c_jk = getCRMTerm(obj, j, k)
            CASPR_log.Assert(j <= obj.numSegments, 'Invalid segment number.');
            % This is checked in segments.getCRMTerm
            %assert(k <= obj.numLinks+1, 'Invalid link number');
            c_jk = obj.segments{j}.getCRMTerm(k);
        end
    end
    
    methods (Static)
        function [segments, attachments] = LoadSegmentsXmlObj(cableName, attachmentXmlObjs, defaultAttachmentRef, bodiesModel)
            numAttachments = attachmentXmlObjs.getLength;
            CASPR_log.Assert(numAttachments >= 2, sprintf('Not enough attachments for cable ''%s'': %d attachment(s) specified', cableName, numAttachments));
            attachments = cell(1, numAttachments);
            segments = cell(1, numAttachments - 1); % Number of segments is one less than the number of attachments
            
            load_base_attachment_pulley = 0;
            
            % Setup the attachments
            for a = 1:numAttachments
                % XML java objects use 0 based index
                attachmentObj = attachmentXmlObjs.item(a-1);
                type = char(attachmentObj.getNodeName);
                
                % Check the type of attachment and then add to the set
                if (strcmp(type, 'attachment'))
                    attachments{a} = CableAttachmentPoint.LoadXmlObj(attachmentObj, defaultAttachmentRef, bodiesModel);
                % Attachment base pulley is a bit special, we need to load
                % the next attachment and then pass this into this one so
                % it can determine its kinematics
                elseif (strcmp(type, 'attachment_base_pulley'))
                    CASPR_log.Assert(a == 1, '''attachment_base_pulley'' can only be used on the base attachment');
                    load_base_attachment_pulley = 1;
                else
                    CASPR_log.Print(sprintf('Unknown cables type: %s', type),CASPRLogLevel.ERROR);
                end
            end
            
            if (load_base_attachment_pulley)
                %attachments{1} = CableAttachmentBasePulley.LoadXmlObj(attachmentXmlObjs(0), attachments{2});
            end
            
            % Using the attachments, setup the segment model
            for s = 1:numAttachments - 1
                segments{s} = CableSegmentModel(attachments{s}, attachments{s+1}, bodiesModel.numLinks);
            end
        end
    end
end
