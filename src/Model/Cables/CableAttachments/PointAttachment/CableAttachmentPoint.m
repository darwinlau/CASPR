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
            
            switch attachment_ref_type
                case CableAttachmentReferenceType.COM
                    ca.r_GA = r_A;
                    if link_num == 0
                        ca.r_PA = ca.r_GA;
                    else
                        r_PG = bodiesModel.bodies{link_num}.r_G;
                        ca.r_PA = r_PG + ca.r_GA;
                    end                    
                case CableAttachmentReferenceType.JOINT
                    ca.r_PA = r_A;
                    if link_num == 0
                        ca.r_GA = ca.r_PA;
                    else
                        r_PG = bodiesModel.bodies{link_num}.r_G;
                        ca.r_GA = ca.r_PA - r_PG;
                    end
                otherwise
                    CASPR_log.Print('CableAttachmentReferenceType type is not defined',CASPRLogLevel.ERROR);
            end
        end
        
        function update(obj, bodyKinematics)
            update@CableAttachmentBase(obj, bodyKinematics);
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
