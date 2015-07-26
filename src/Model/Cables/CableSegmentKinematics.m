classdef CableSegmentKinematics < handle
    %CABLEKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = private)
        numLinks = -1
    end
    
    properties
        attachmentsLocal = {}       % Cell array of relative attachment locations ^kr_{o_kA_{ijk}} for cable i segment j in local frame
        attachmentsAbs = {}         % Cell array of absolute attachment locations ^kr_{OA_{ijk}} for cable i segment j in local frame
        segmentVector = []          % vectors \mathbf{l}_{ij} in frame {0}
        CRM = []                    % Cable Routing Matrix for this cable and segment
    end
    
    properties (Dependent)
        length
    end
        
    methods
        function ck = CableSegmentKinematics(numLinks, sLink, sLoc, eLink, eLoc)
            assert(sLink <= numLinks+1, '"sLink" exceeds total number of links');
            assert(eLink <= numLinks+1, '"eLink" exceeds total number of links');
            
            ck.numLinks = numLinks;
            ck.segmentVector = [0;0;0];
            ck.CRM = zeros(1, numLinks+1);
            ck.attachmentsLocal = cell(1, numLinks+1);
            ck.attachmentsLocal(:) = {[0;0;0]};
            ck.attachmentsAbs = cell(1, numLinks+1);
            ck.attachmentsAbs(:) = {[0;0;0]};
            
            ck.CRM(sLink+1) = -1;
            ck.CRM(eLink+1) = 1;
            ck.attachmentsLocal{sLink+1} = sLoc;
            ck.attachmentsLocal{eLink+1} = eLoc;
        end
        
        function c_k = getCRMTerm(obj, k)
            assert(k <= obj.numLinks+1, 'Invalid link number.');
            
            c_k = obj.CRM(k);
        end
        
        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done
        function clear(obj)
            obj.CRM = zeros(1, obj.numLinks+1);
            obj.attachmentsLocal(:) = {[0;0;0]};
            obj.attachmentsAbs(:) = {[0;0;0]};
        end
        
        function value = get.length(obj)
            value = sqrt(sum(obj.segmentVector.^2));
%             value = norm(obj.segmentVector);
        end
    end
    
end

