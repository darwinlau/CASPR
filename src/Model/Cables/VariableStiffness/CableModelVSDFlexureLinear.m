% Model for a variable stiffness (VS) cable actuator with a linear
% stiffness-displacement relationship. 
%
% Please cite the following paper when using this VS actuator:
% K. Yang, G. Yang, J. Wang, T. Zheng, and W. Yang, "Design Analysis of a
% 3-DOF Cable-driven Variable-stiffness Joint Module", in Proc. IEEE Conf.
% Robot. Biomimetics (ROBIO), pp. 529-534, 2015.
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%	The cable actuator is assumed to be a series of the linear cable spring
%	(K_cable) and a variable stiffness actuator (K_vs). The stiffness of
%	the VS actuator is dependent on the cable force, where there is a
%	linear stiffness-displacement relationship. 
classdef CableModelVSDFlexureLinear < CableModelBase      
    properties (SetAccess = private)
        K_cable = 1;
        forceDeformationRelationCoeff = [4,1,2]; % default values
    end
    
    properties (Dependent)
        K
        K_vsd
    end
        
    methods 
        function ck = CableModelVSDFlexureLinear(name, numLinks)
            ck@CableModelBase(name, numLinks);
        end
        
        function update(obj, bodyModel)
            update@CableModelBase(obj, bodyModel);
        end
        
        function value = get.K(obj)
            value = 1/(1/obj.K_cable + 1/obj.K_vsd);
        end
        
        function value = get.K_vsd(obj)
            % Need to handle the case of no cable force assigned somehow
            if (obj.force == CableModelBase.INVALID_FORCE)
            end
            a = obj.forceDeformationRelationCoeff(1);
            b = obj.forceDeformationRelationCoeff(2);
            c = obj.forceDeformationRelationCoeff(3);
            value = sqrt(4*a*obj.force + (b^2 - 4*a*c));
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(xmlobj, bodiesModel)
            % <cable_vsd_flexure_linear> tag
            name = char(xmlobj.getAttribute('name'));
            attachRefString = char(xmlobj.getAttribute('attachment_reference'));
            CASPR_log.Assert(~isempty(attachRefString), 'Invalid <cable_vsd_flexure_linear> XML format: attachment_reference field empty');
            if (strcmp(attachRefString, 'joint'))
                attachmentRef = CableAttachmentReferenceType.JOINT;
            elseif (strcmp(attachRefString, 'com'))
                attachmentRef = CableAttachmentReferenceType.COM;
            else
                CASPR_log.Print(sprintf('Unknown cableAttachmentReference type: %s', attachRefString),CASPRLogLevel.ERROR);
            end
            
            % Generate an ideal cable object
            c = CableModelVSDFlexureLinear(name, bodiesModel.numLinks);
            
            % <properties> tag
            propertiesObj = xmlobj.getElementsByTagName('properties').item(0);
            % <K_cable>
            c.K_cable = str2double(propertiesObj.getElementsByTagName('K_cable').item(0).getFirstChild.getData);
            % <vsd_force_deformation_relation>: quadratic relationship
            % between force and deformation [a b c]
            c.forceDeformationRelationCoeff = XmlOperations.StringToVector3(char(propertiesObj.getElementsByTagName('vsd_force_deformation_relation').item(0).getFirstChild.getData));
            % <force_min>
            c.forceMin = str2double(propertiesObj.getElementsByTagName('force_min').item(0).getFirstChild.getData);
            % <force_max>
            c.forceMax = str2double(propertiesObj.getElementsByTagName('force_max').item(0).getFirstChild.getData);
            
            % <attachments> tag
            attachmentObjs = xmlobj.getElementsByTagName('attachments').item(0).getElementsByTagName('attachment');
            CASPR_log.Assert(~isempty(name), 'No name specified for this cable');
            CASPR_log.Assert(attachmentObjs.getLength >= 2, sprintf('Not enough attachments for cable ''%s'': %d attachment(s) specified', name, attachmentObjs.getLength));
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

