% Model for a linearly spring cable
% 
% Author        : Darwin LAU
% Created       : 2015
% Description	:
%	Extending from the ideal cable model, it is assumed that the cable can
%	be modelled like a linear spring, either because of its own stretching
%	or that a spring is put in series in the cable (or both). The main
%	parameter that needs to be specified is the overall cable stiffness K.
classdef CableModelLinearSpring < CableModelBase        
    properties (SetAccess = private)
        K_cable
    end
    
    properties (Dependent)
        K
    end
    
    methods 
        function ck = CableModelLinearSpring(name, numLinks)
            ck@CableModelBase(name, numLinks);
        end
        
        function update(obj, bodyModel)
            update@CableModelBase(obj, bodyModel);
        end
        
        function value = get.K(obj)
            value = obj.K_cable/obj.length;
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(xmlobj, bodiesModel)
            % <cable_linear_spring> tag
            name = char(xmlobj.getAttribute('name'));
            attachRefString = char(xmlobj.getAttribute('attachment_reference'));
            
            CASPR_log.Assert(~isempty(name), 'No name specified for this cable');
            CASPR_log.Assert(~isempty(attachRefString), 'Invalid <cable_linear_spring> XML format: attachment_reference field empty');
            
            if (strcmp(attachRefString, 'joint'))
                attachmentRef = CableAttachmentReferenceType.JOINT;
            elseif (strcmp(attachRefString, 'com'))
                attachmentRef = CableAttachmentReferenceType.COM;
            else
                CASPR_log.Print(sprintf('Unknown cableAttachmentReference type: %s', attachRefString),CASPRLogLevel.ERROR);
            end
                        
            % <properties> tag
            propertiesObj = xmlobj.getElementsByTagName('properties').item(0);
            % Generate an ideal cable object
            c = CableModelLinearSpring(name, bodiesModel.numLinks);
            
            % <K>
            c.K_cable = str2double(propertiesObj.getElementsByTagName('K').item(0).getFirstChild.getData);
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
            [c.segments, c.attachments] = CableModelBase.LoadSegmentsXmlObj(name, attachmentObjs, attachmentRef, bodiesModel);     
        end
    end
end

