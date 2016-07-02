% Abstract class for the kinematics and dynamics of a single cable
% 
% Please cite the following paper when using this for multilink cable
% robots:
% D. Lau, D. Oetomo, and S. K. Halgamuge, "Generalized Modeling of
% Multilink Cable-Driven Manipulators with Arbitrary Routing Using the
% Cable-Routing Matrix," IEEE Trans. Robot., vol. 29, no. 5, pp. 1102�1113,
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
classdef (Abstract) CableModelBase < handle
    properties (Constant)
        INVALID_FORCE = -1
    end
    
    properties 
        % Actual cable force, use -1 to represent invalid forces
        force = CableModelBase.INVALID_FORCE;
    end
    
    properties (SetAccess = protected)
        segments = {}               % Cell array of CableSegmentKinematics
        isActive = true             % Whether the cable is in passive state
        % Minimum and maximum allowable cable force
        forceMin
        forceMax
    end    
    
    properties (SetAccess = private)
        name        = '';                  % Cable name
        numLinks    = -1;
        numSegments = 0;
    end
    
    properties (Dependent)
        length 
    end
    
    properties (Abstract, Dependent)
        % Stiffness of the cable
        K
    end
        
    methods
        function ck = CableModelBase(name, numLinks)
            ck.name = name;
            ck.numLinks = numLinks;
        end
                
        % Add first segment and update CRM
        function addSegment(obj, sLink, sLoc, eLink, eLoc, bodiesKin, attachmentRefType)
            segmentId = obj.numSegments + 1;
            obj.segments{segmentId} = CableSegmentModel(obj.numLinks, sLink, sLoc, eLink, eLoc, bodiesKin, attachmentRefType);
            obj.numSegments = segmentId;
        end
        
        % NOT SURE HOW THIS IS USED YET, but just a demo of what can be
        % done
        function clear(obj)
            obj.numSegments = 0;
            obj.segments = {};
        end
        
        function update(obj, bodyKinematics)
            for j = 1:obj.numSegments
                % cycle through links 0 to p, linkNum = k-1
                segment = obj.segments{j};
                segment.segmentVector = [0;0;0];
                for k = 1:obj.numLinks+1
                    CRMTerm = obj.getCRMTerm(j,k);
                    % First : compute absolute attachment locations
                    if CRMTerm ~= 0
                        % k == 1 is base link
                        if k == 1
                            segment.r_OA{k} = segment.r_PA{k};
                        else
                            % bodies{k-1} because bodyNum = k - 1;
                            segment.r_OA{k} = bodyKinematics.bodies{k-1}.r_OG + segment.r_GA{k};
                        end
                    end
                    % Second : compute cable segment vectors
                    % k == 1 is base link
                    if k == 1
                        segment.segmentVector = segment.segmentVector + CRMTerm*segment.r_OA{k};
                    else
                        segment.segmentVector = segment.segmentVector + CRMTerm*(bodyKinematics.bodies{k-1}.R_0k*segment.r_OA{k});
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
