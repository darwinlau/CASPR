% Abstract class for the kinematics of a single cable
% 
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102–1113,
% Oct. 2013.
% 
% Author        : Darwin LAU
% Created       : 2011
% Description	:
%	Data structure that contains the kinematics of a multisegment cable. It
%	provides:
%       - the ability to add and remove segments
%       - update the kinematics (i.e. attachment locations) of each segment
%       - access the CRM term
classdef (Abstract) CableKinematics < handle
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
        function addSegment(obj, sLink, sLoc, eLink, eLoc, bodiesKin, attachmentRefType)
            segmentId = obj.numSegments + 1;
            obj.segments{segmentId} = CableSegmentKinematics(obj.numLinks, sLink, sLoc, eLink, eLoc, bodiesKin, attachmentRefType);
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
                            obj.segments{j}.r_OA{k} = obj.segments{j}.r_PA{k};
                        else
                            % bodies{k-1} because bodyNum = k - 1;
                            obj.segments{j}.r_OA{k} = bodyKinematics.bodies{k-1}.r_OG + obj.segments{j}.r_GA{k};
                        end
                    end
                    % Second : compute cable segment vectors
                    % k == 1 is base link
                    if k == 1
                        obj.segments{j}.segmentVector = obj.segments{j}.segmentVector + obj.getCRMTerm(j,k)*obj.segments{j}.r_OA{k};
                    else
                        obj.segments{j}.segmentVector = obj.segments{j}.segmentVector + obj.getCRMTerm(j,k)*(bodyKinematics.bodies{k-1}.R_0k*obj.segments{j}.r_OA{k});
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

