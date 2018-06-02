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
        K_cable
        forceDeformationRelationCoeff 
    end
    
    properties (Dependent)
        K
        K_vsd
    end
        
    methods 
        function ck = CableModelVSDFlexureLinear(name, numLinks, K_cable, forceDeformationRelationCoeff)
            % Constructor
            ck@CableModelBase(name, numLinks);
            ck.K_cable = K_cable;
            ck.forceDeformationRelationCoeff = forceDeformationRelationCoeff;
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
            
            CASPR_log.Assert(~isempty(name), 'No name specified for this cable');
            CASPR_log.Assert(~isempty(attachRefString), 'Invalid <cable_vsd_flexure_linear> XML format: attachment_reference field empty');
            
            if (strcmp(attachRefString, 'joint'))
                attachmentRef = CableAttachmentReferenceType.JOINT;
            elseif (strcmp(attachRefString, 'com'))
                attachmentRef = CableAttachmentReferenceType.COM;
            else
                CASPR_log.Print(sprintf('Unknown cableAttachmentReference type: %s', attachRefString),CASPRLogLevel.ERROR);
            end
            
            % <properties> tag
            propertiesObj = xmlobj.getElementsByTagName('properties').item(0);
            % <K_cable>
            K_cable = str2double(propertiesObj.getElementsByTagName('K_cable').item(0).getFirstChild.getData);
            % <vsd_force_deformation_relation>: quadratic relationship
            % between force and deformation [a b c]
            forceDeformationRelationCoeff = XmlOperations.StringToVector3(char(propertiesObj.getElementsByTagName('vsd_force_deformation_relation').item(0).getFirstChild.getData));
            % Generate an ideal cable object
            c = CableModelVSDFlexureLinear(name, bodiesModel.numLinks, K_cable, forceDeformationRelationCoeff);
            
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

