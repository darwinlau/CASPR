% The simplest type of cable attachment where it is only a single point
% 
% Author        : Darwin LAU
% Created       : 2016
% Description	:
classdef CableAttachmentPoint < CableAttachmentBase
    methods
        function ca = CableAttachmentPoint(link_num, r_A, attachment_ref_type, bodiesModel)
            numLinks = bodiesModel.numLinks;
            CASPR_log.Assert(link_num <= numLinks, '"link_num" exceeds total number of links');
            ca.link_num = link_num;
            
            if (link_num == 0)
                ca.r_PG = [0; 0; 0];
            else
                ca.r_PG = bodiesModel.bodies{link_num}.r_G;
            end
            ca.updateAttachmentLocation(r_A, attachment_ref_type);
        end
        
        function update(obj, bodyKinematics)
            update@CableAttachmentBase(obj, bodyKinematics);
        end
        
        function updateAttachmentLocation(obj, r_A, attachment_ref_type)                
            switch attachment_ref_type
                case CableAttachmentReferenceType.COM
                    obj.r_GA = r_A;                
                case CableAttachmentReferenceType.JOINT
                    % For link 0, r_PG = [0; 0; 0] anyway
                    obj.r_GA = r_A - obj.r_PG;
                otherwise
                    CASPR_log.Print('CableAttachmentReferenceType type is not defined',CASPRLogLevel.ERROR);
            end
        end
    end
    
    methods (Static)
        function ca = LoadXmlObj(xmlObj, defaultAttachmentRef, bodiesModel)
            link_num = str2double(xmlObj.getElementsByTagName('link').item(0).getFirstChild.getData);
            r_A = XmlOperations.StringToVector3(char(xmlObj.getElementsByTagName('location').item(0).getFirstChild.getData));
            ca = CableAttachmentPoint(link_num, r_A, defaultAttachmentRef, bodiesModel);  
        end
    end
end
