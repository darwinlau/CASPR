% Model for a variable stiffness (VS) cable actuator using torsion spring
% coils.
% 
% Author        : Chen SONG
% Created       : 2018
% Description	:
%	Variable stiffness actuator where the stiffness is a polynomial of
%	cable tensions
classdef CableModelVSDPolynomial < CableModelBase              
    properties (SetAccess = private)
        K_cable
        forceStiffnessRelationCoeff
        stiffnessPolyCoeffNumber
    end
    
    properties (Dependent)
        K
        K_vsd
    end
    
    methods 
        function ck = CableModelVSDPolynomial(name, numLinks, K_cable, forceStiffnessRelationCoeff, stiffnessPolyCoeffNumber)
            ck@CableModelBase(name, numLinks);
            ck.K_cable = K_cable;
            ck.forceStiffnessRelationCoeff  =   forceStiffnessRelationCoeff;
            ck.stiffnessPolyCoeffNumber     =   stiffnessPolyCoeffNumber;
        end
        
        function update(obj, bodyModel)
            update@CableModelBase(obj, bodyModel);
        end
        
        function value = get.K(obj)
            value = 1/(obj.length/obj.K_cable + 1/obj.K_vsd);
        end
        
        function value = get.K_vsd(obj)
            % Need to handle the case of no cable force assigned somehow
            if (obj.force == CableModelBase.INVALID_FORCE)
            end
            value = 0;
            for i = 1:obj.stiffnessPolyCoeffNumber
                value = value*obj.force + obj.forceStiffnessRelationCoeff(obj.stiffnessPolyCoeffNumber - i + 1);
            end
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(xmlobj, bodiesModel)
            % <cable_vsd_polynomial> tag
            name = char(xmlobj.getAttribute('name'));
            attachRefString = char(xmlobj.getAttribute('attachment_reference'));
            
            CASPR_log.Assert(~isempty(name), 'No name specified for this cable');
            CASPR_log.Assert(~isempty(attachRefString), 'Invalid <cable_vsd_polynomial> XML format: attachment_reference field empty');
            
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
            % <num_stiffness_polynomial_coeff>
            stiffnessPolyCoeffNumber = str2double(propertiesObj.getElementsByTagName('num_stiffness_polynomial_coeff').item(0).getFirstChild.getData);
            % <stiffness_poly_coeff> -- hard coded for now
%             forceStiffnessRelationCoeff = str2double(propertiesObj.getElementsByTagName('torsion_spring_stiffness').item(0).getFirstChild.getData);
            forceStiffnessRelationCoeff = [5.229e3 1.433e2 -3.226e-2 1.77e-2];
            % Generate an general VSD cable object
            c = CableModelVSDPolynomial(name, bodiesModel.numLinks, K_cable, forceStiffnessRelationCoeff, stiffnessPolyCoeffNumber);
            
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

