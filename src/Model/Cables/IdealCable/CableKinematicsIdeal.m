classdef CableKinematicsIdeal < CableKinematics
    %CABLEKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
        
    methods 
        function ck = CableKinematicsIdeal(name, numLinks)
            ck@CableKinematics(name, numLinks);
        end
        
        function update(obj, bodyKinematics)
            update@CableKinematics(obj, bodyKinematics);
        end
    end
    
    methods (Static)
        function c = LoadXmlObj(xmlobj, numLinks)
            name = char(xmlobj.getAttribute('name'));
            c = CableKinematicsIdeal(name, numLinks);
            
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
                
                c.addSegment(sLink, sLoc, eLink, eLoc);
                
                sLink = eLink;
                sLoc = eLoc;                
            end            
        end
    end
end

