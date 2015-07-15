classdef (Abstract) CableKinematics < handle
    %CABLEKINEMATICS Summary of this class goes here
    %   Detailed explanation goes here
    properties
        segments = {}               % Cell array of CableSegmentKinematics
        name = '';                  % Cable name
    end
    
    properties (SetAccess = private)
        numLinks = -1
        numSegments = 0
    end
    
    properties (Dependent)
        length 
    end
        
    methods
        function ck = CableKinematics(name, numLinks)
            ck.name = name;
            ck.numLinks = numLinks;
        end
                
        % Add first segment and update CRM
        function addSegment(obj, sLink, sLoc, eLink, eLoc)
            segmentId = obj.numSegments + 1;
            obj.segments{segmentId} = CableSegmentKinematics(obj.numLinks, sLink, sLoc, eLink, eLoc);
            obj.numSegments = segmentId;
        end
        
        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done
        function clearSegments(obj)
            obj.numSegments = 0;
            obj.segments = {};
        end
        
        function update(obj, bodyKinematics)
            for j = 1:obj.numSegments
                % cycle through links 0 to p, linkNum = k-1
                obj.segments{j}.segmentVector = [0;0;0];
                for k = 1:obj.numLinks+1
                    % First : compute absolute attachment locations
                    if obj.getCRMTerm(j,k) ~= 0
                        % k == 1 is base link
                        if k == 1
                            obj.segments{j}.attachmentsAbs{k} = obj.segments{j}.attachmentsLocal{k};
                        else
                            % bodies{k-1} because bodyNum = k - 1;
                            obj.segments{j}.attachmentsAbs{k} = bodyKinematics.bodies{k-1}.r_OG + obj.segments{j}.attachmentsLocal{k};
                        end
                    end
                    % Second : compute cable segment vectors
                    % k == 1 is base link
                    if k == 1
                        obj.segments{j}.segmentVector = obj.segments{j}.segmentVector + obj.getCRMTerm(j,k)*obj.segments{j}.attachmentsAbs{k};
                    else
                        obj.segments{j}.segmentVector = obj.segments{j}.segmentVector + obj.getCRMTerm(j,k)*(bodyKinematics.bodies{k-1}.R_0k*obj.segments{j}.attachmentsAbs{k});
                    end
                end
            end
        end
        
        function value = get.length(obj)
            value = 0;
            for j = 1:obj.numSegments
                value = value + obj.segments{j}.length;
            end
            %value = sum(obj.segments{:}.length);
        end
        
        function c_jk = getCRMTerm(obj, j, k)
            assert(j <= obj.numSegments, 'Invalid segment number.');
            % This is checked in segments.getCRMTerm
            %assert(k <= obj.numLinks+1, 'Invalid link number');
            
            c_jk = obj.segments{j}.getCRMTerm(k);
        end
    end
    
end

