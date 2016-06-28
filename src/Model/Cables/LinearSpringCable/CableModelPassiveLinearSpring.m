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
            assert(~isempty(attachRefString), 'Invalid <cable_passive_linear_spring> XML format: attachment_reference field empty');
            if (strcmp(attachRefString, 'joint'))
                attachmentRef = CableAttachmentReferenceType.JOINT;
            elseif (strcmp(attachRefString, 'com'))
                attachmentRef = CableAttachmentReferenceType.COM;
            else
                error('Unknown cableAttachmentReference type: %s', attachRefString);
            end
            
            % Generate an object
            c = CableModelPassiveLinearSpring(name, bodiesModel.numLinks);
            
            % <properties> tag
            propertiesObj = xmlobj.getElementsByTagName('properties').item(0);
            % <K>
            c.K_cable = str2double(propertiesObj.getElementsByTagName('K').item(0).getFirstChild.getData);
            % <l0>
            c.l_0 = str2double(propertiesObj.getElementsByTagName('l0').item(0).getFirstChild.getData);
            
            % <attachments> tag
            attachmentObjs = xmlobj.getElementsByTagName('attachments').item(0).getElementsByTagName('attachment');
            assert(~isempty(name), 'No name specified for this cable');
            assert(attachmentObjs.getLength >= 2, sprintf('Not enough attachments for cable ''%s'': %d attachment(s) specified', name, attachmentObjs.getLength));
            % Beginning attachment of segment 1
            attachmentObj = attachmentObjs.item(0);
            sLink = str2double(attachmentObj.getElementsByTagName('link').item(0).getFirstChild.getData);
            sLoc = XmlOperations.StringToVector3(char(attachmentObj.getElementsByTagName('location').item(0).getFirstChild.getData));
            for a = 2:attachmentObjs.getLength
                attachmentObj = attachmentObjs.item(a-1);
                eLink = str2double(attachmentObj.getElementsByTagName('link').item(0).getFirstChild.getData);
                eLoc = XmlOperations.StringToVector3(char(attachmentObj.getElementsByTagName('location').item(0).getFirstChild.getData));
                
                c.addSegment(sLink, sLoc, eLink, eLoc, bodiesModel, attachmentRef);
                
                sLink = eLink;
                sLoc = eLoc;                
            end            
        end
    end
end

