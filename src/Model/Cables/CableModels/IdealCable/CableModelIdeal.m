% Model for an ideal (massless and rigid) cable
% 
% Author        : Darwin LAU
% Created       : 2011
% Description	:
%	This is the simplest type of cable that is massless and rigid. It also
%	assumes a straight-line model between attachment points. As such, the
%	only parameters that govern the kinematics of the ideal cable are the
%	attachment locations of cables at each link.
classdef CableModelIdeal < CableModelBase     
    properties (Dependent)
        K
    end
    
    methods 
        function ck = CableModelIdeal(name, numLinks)
            ck@CableModelBase(name, numLinks);
        end
        
        function update(obj, bodyModel)
            update@CableModelBase(obj, bodyModel);
        end
        
        function value = get.K(~)
            % Ideal cables have infinite stiffness
            value = Inf;
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(xmlobj, bodiesModel)
            % <cable_ideal> tag
            name = char(xmlobj.getAttribute('name'));
            attachRefString = char(xmlobj.getAttribute('attachment_reference'));
            
            CASPR_log.Assert(~isempty(name), 'No name specified for this cable');
            CASPR_log.Assert(~isempty(attachRefString), 'Invalid <cable_ideal> XML format: attachment_reference field empty');
            
            if (strcmp(attachRefString, 'joint'))
                defaultAttachmentRef = CableAttachmentReferenceType.JOINT;
            elseif (strcmp(attachRefString, 'com'))
                defaultAttachmentRef = CableAttachmentReferenceType.COM;
            else
                error('Unknown cableAttachmentReference type: %s', attachRefString);
            end
                        
            % <properties> tag
            propertiesObj = xmlobj.getElementsByTagName('properties').item(0);
            % Generate an ideal cable object
            c = CableModelIdeal(name, bodiesModel.numLinks);
            
            % <force_min>
            c.forceMin = str2double(propertiesObj.getElementsByTagName('force_min').item(0).getFirstChild.getData);
            % <force_max>
            c.forceMax = str2double(propertiesObj.getElementsByTagName('force_max').item(0).getFirstChild.getData);
            % <diameter>
            lol = propertiesObj.getElementsByTagName('diameter');
            if (lol.getLength() ~= 0) % exist
                c.diameter = str2double(propertiesObj.getElementsByTagName('diameter').item(0).getFirstChild.getData);
            end
            
            % <attachments> tag
            attachmentObjs = xmlobj.getElementsByTagName('attachments').item(0).getChildNodes();            
            [c.segments, c.attachments] = CableModelBase.LoadSegmentsXmlObj(name, attachmentObjs, defaultAttachmentRef, bodiesModel);   
        end
    end
end

