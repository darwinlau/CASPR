% Model for a passive linear spring
% 
% Author        : Darwin LAU
% Created       : 2015
% Description	:
%   This cable model is an always passive spring element. 
%   TODO: Need to consider the scenario of multiple links
classdef CableModelPassiveLinearSpring < CableModelBase        
    properties (SetAccess = private)
        K_cable
        l_0
    end
    
    properties (Dependent)
        K
    end
    
    methods 
        function ck = CableModelPassiveLinearSpring(name, numLinks)
            ck@CableModelBase(name, numLinks);
            ck.isActive = false;
        end
        
        function update(obj, bodyModel)
            update@CableModelBase(obj, bodyModel);
            obj.force = obj.K_cable * (obj.length - obj.l_0);
        end
        
        function value = get.K(obj)
            value = obj.K_cable/obj.length;
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(xmlobj, bodiesModel)
            % <cable_passive_linear_spring> tag
            name = char(xmlobj.getAttribute('name'));
            attachRefString = char(xmlobj.getAttribute('attachment_reference'));
            
            CASPR_log.Assert(~isempty(name), 'No name specified for this cable');
            CASPR_log.Assert(~isempty(attachRefString), 'Invalid <cable_passive_linear_spring> XML format: attachment_reference field empty');
            
            if (strcmp(attachRefString, 'joint'))
                attachmentRef = CableAttachmentReferenceType.JOINT;
            elseif (strcmp(attachRefString, 'com'))
                attachmentRef = CableAttachmentReferenceType.COM;
            else
                CASPR_log.Print(sprintf('Unknown cableAttachmentReference type: %s', attachRefString),CASPRLogLevel.ERROR);
            end
                        
            % <properties> tag
            propertiesObj = xmlobj.getElementsByTagName('properties').item(0);
            % Generate an object
            c = CableModelPassiveLinearSpring(name, bodiesModel.numLinks);
            
            % <K>
            c.K_cable = str2double(propertiesObj.getElementsByTagName('K').item(0).getFirstChild.getData);
            % <l0>
            c.l_0 = str2double(propertiesObj.getElementsByTagName('l0').item(0).getFirstChild.getData);
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

