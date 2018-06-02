% Model for a variable stiffness (VS) cable actuator using torsion spring
% coils.
%
% Please cite the following paper when using this VS actuator:
% S.H. Yeo, G. Yang, and W.B. Lim, "Design and Analysis of Cable-Driven
% Manipulators with Variable Stiffness", Mech. Mach. Theory, Vol. 69, pp.
% 230-244, 2013.
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
%	Variable stiffness actuator where the stiffness is a function of the
%	cable force. The stiffness to displacement, and hence force,
%	relationships are non-linear for this type of VSD.
classdef CableModelVSDTorsionSpring < CableModelBase              
    properties (SetAccess = private)
        K_cable
        numSprings
        springStiffness              % Variable 'K_theta' in the paper
        springLength                 % Variable 'd' in the paper
    end
    
    properties (Dependent)
        K
        K_vsd
    end
    
    methods 
        function ck = CableModelVSDTorsionSpring(name, numLinks, K_cable, numSprings, springStiffness, springLength)
            ck@CableModelBase(name, numLinks);
            ck.K_cable = K_cable;
            ck.numSprings = numSprings;
            ck.springStiffness = springStiffness;
            ck.springLength = springLength;
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
            C_theta = (obj.force * obj.springLength)/(2 * obj.numSprings * obj.springStiffness);
            theta = (1.571 * C_theta^3 + 1.251 * C_theta^2 + 0.696 * C_theta - 0.0000158)/(C_theta^3 + 1.793 * C_theta^2 + 1.271 * C_theta + 0.695);
            L = 2 * obj.springLength * sin(theta);
            value = ((4 * obj.numSprings * obj.springStiffness) / (4 * obj.springLength^2 - L^2)) * ( 1 + L * asin(L/(2*obj.springLength))/sqrt(4 * obj.springLength^2 - L^2));
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(xmlobj, bodiesModel)
            % <cable_vsd_torsion_spring> tag
            name = char(xmlobj.getAttribute('name'));
            attachRefString = char(xmlobj.getAttribute('attachment_reference'));
            
            CASPR_log.Assert(~isempty(name), 'No name specified for this cable');
            CASPR_log.Assert(~isempty(attachRefString), 'Invalid <cable_vsd_torsion_spring> XML format: attachment_reference field empty');
            
            if (strcmp(attachRefString, 'joint'))
                attachmentRef = CableAttachmentReferenceType.JOINT;
            elseif (strcmp(attachRefString, 'com'))
                attachmentRef = CableAttachmentReferenceType.COM;
            else
                CASPR_log.Print(sprintf('Unknown cableAttachmentReference type: %s', attachRefString),CASPRLogLevel.ERROR);
            end
            
            % <properties> tag
            propertiesObj = xmlobj.getElementsByTagName('properties').item(0);
            % <K>
            K_cable = str2double(propertiesObj.getElementsByTagName('K_cable').item(0).getFirstChild.getData);
            % <num_torsion_springs>
            numSprings = str2double(propertiesObj.getElementsByTagName('num_torsion_springs').item(0).getFirstChild.getData);
            % <torsion_spring_stiffness>
            springStiffness = str2double(propertiesObj.getElementsByTagName('torsion_spring_stiffness').item(0).getFirstChild.getData);
            % <torsion_spring_length>
            springLength = str2double(propertiesObj.getElementsByTagName('torsion_spring_length').item(0).getFirstChild.getData);
            % Generate an ideal cable object
            c = CableModelVSDTorsionSpring(name, bodiesModel.numLinks, K_cable, numSprings, springStiffness, springLength);
            
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

