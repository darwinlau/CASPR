% Class for avoiding interference (cable-link, link-link)
%
% Author        : Dominic Chan
% Created       : 2019
% Description   : This formulation focus on generality, not efficiency
% *** Distances are evaluated based on updated body and cable info
% *** Hence, it must be run in DEFAULT mode
classdef AvoidInterference < AvoidanceBase
    properties        
        buffer_cablelink    % Scalar - buffer between cable and link
        buffer_linklink     % Scalar - buffer between link and link
        index_cablelink     % nx1 Vector - n link numbers that need avoidance
        index_linklink      % nx2 Matrix - n link number pairs that need avoidance
        
        cablelinkDist       % (nm)x1 Vector - distance between cable-link pairs
        linklinkDist        % nx1 Vector - distance between link-link pairs
        worst_cableLink     % Worst case index for cable link
        worst_linkLink      % Worst case index for link link
    end

    methods
        % Constructor
        function ai = AvoidInterference(cdpr, dt, k_q, epsilon, ...
                buffer_cablelink, index_cablelink, buffer_linklink, index_linklink)            
            ai@AvoidanceBase(cdpr, dt, k_q, epsilon);                       
            ai.buffer_cablelink = buffer_cablelink; 
            ai.index_cablelink  = index_cablelink;
            ai.cablelinkDist    = Inf(length(index_cablelink)*cdpr.numCables,1);
            if nargin > 6
                CASPR_log.Assert(nargin == 8, 'Please provide both the buffer and index for link-link interference');
                ai.buffer_linklink  = buffer_linklink; 
                ai.index_linklink   = index_linklink;
                ai.linklinkDist     = zeros(size(index_linklink,1),1);
            else
                ai.buffer_linklink  = []; 
                ai.index_linklink   = [];
                ai.linklinkDist     = [];
            end
            ai.worst_cableLink      = [];
            ai.worst_linkLink       = [];
        end     
        
        % Init function
        function initVariables(obj)
            obj.worst_cableLink     = [];
            obj.worst_linkLink      = [];
        end
                
        % Evaluate function
        function value = evaluate(obj, q, q_d) 
            % Update model
            obj.cdpr.update(q, q_d, zeros(obj.cdpr.numDofs,1), zeros(obj.cdpr.numDofs,1));
            % Evaluate distances
            minCableLinkDist    = obj.evaluateCableLinkDist(); 
            if isempty(minCableLinkDist)
                minCableLinkDist = Inf;
            end
            
            if ~isempty(obj.buffer_linklink)
                obj.evaluateLinkLinkDist();
                minLinkLinkDist = min(obj.linklinkDist); 
            else
                minLinkLinkDist = Inf;
            end            
            value = min(minCableLinkDist, minLinkLinkDist);
        end
        
        % Distance function (cable-link)
        % - Evaluate distance between cable and link
        function minCableLinkDist = evaluateCableLinkDist(obj)
            if isempty(obj.worst_cableLink)
                % Loop through each link in index_cablelink
                for i = 1:length(obj.index_cablelink)
                    linkindex = obj.index_cablelink(i);               
                    link_start = obj.cdpr.bodyModel.bodies{linkindex}.R_0k*obj.cdpr.bodyModel.bodies{linkindex}.r_OP;                
                    link_end = obj.cdpr.bodyModel.bodies{linkindex}.R_0k*obj.cdpr.bodyModel.bodies{linkindex}.r_OPe;
                    % Loop through each cable
                    for j = 1:obj.cdpr.numCables
                        cable_start = obj.cdpr.cableModel.cables{1,j}.attachments{1,1}.r_OA;
                        cable_end = obj.cdpr.cableModel.cables{1,j}.attachments{1,2}.r_OA;
                        obj.cablelinkDist((i-1)*obj.cdpr.numCables+j) = ...
                            DistBetween2Segment(link_start, link_end, cable_start, cable_end);
                    end
                end
                [minCableLinkDist, obj.worst_cableLink] = min(obj.cablelinkDist);               
            else
                % Just evaluate the worst case
                linkindex  = obj.index_cablelink(ceil(obj.worst_cableLink/obj.cdpr.numCables));
                cableindex = mod(obj.worst_cableLink,obj.cdpr.numCables);
                link_start = obj.cdpr.bodyModel.bodies{linkindex}.R_0k*obj.cdpr.bodyModel.bodies{linkindex}.r_OP;
                link_end = obj.cdpr.bodyModel.bodies{linkindex}.R_0k*obj.cdpr.bodyModel.bodies{linkindex}.r_OPe;
                cable_start = obj.cdpr.cableModel.cables{1,cableindex}.attachments{1,1}.r_OA;
                cable_end = obj.cdpr.cableModel.cables{1,cableindex}.attachments{1,2}.r_OA;
                % Worst case dist
                minCableLinkDist = DistBetween2Segment(link_start, link_end, cable_start, cable_end);
            end
        end
        
        % Distance function (link-link)
        % - Evaluate distance between links
        function evaluateLinkLinkDist(obj)
            % Loop through each link pair in index_linklink
            for i = 1:size(obj.index_linklink,1)
                % Link in 1st column
                link1index = obj.index_linklink(i,1);               
                link1_start = obj.cdpr.bodyModel.bodies{link1index}.R_0k*obj.cdpr.bodyModel.bodies{link1index}.r_OP;                
                link1_end = obj.cdpr.bodyModel.bodies{link1index}.R_0k*obj.cdpr.bodyModel.bodies{link1index}.r_OPe;
                % Link in 2nd column
                link2index = obj.index_linklink(i,2);               
                link2_start = obj.cdpr.bodyModel.bodies{link2index}.R_0k*obj.cdpr.bodyModel.bodies{link2index}.r_OP;                
                link2_end = obj.cdpr.bodyModel.bodies{link2index}.R_0k*obj.cdpr.bodyModel.bodies{link2index}.r_OPe;
                % Distance
                obj.linklinkDist(i) = DistBetween2Segment(link1_start, ...
                    link1_end, link2_start, link2_end);
            end
        end
        
        % Triggers
        % - Evaluate distances here and save them
        function flag = isAvoidanceObjective(obj, ~, ~)
            flag = false;
            % Evaluate Distance 
            min_dist = obj.evaluateCableLinkDist();            
            if min_dist < obj.buffer_cablelink
                flag = true;
                return;
            end
            if ~isempty(obj.buffer_linklink)
                obj.evaluateLinkLinkDist();
                disp(min(obj.linklinkDist));
                if min(obj.linklinkDist) < obj.buffer_linklink
                    flag = true;                    
                end
            end            
        end
        function flag = isAvoidanceConstraints(obj, ~, ~)
            flag = false;
            % Default hard limit for now (1cm)
            hard_dist = 1e-2;
            % Evaluate Distance 
            min_dist = obj.evaluateCableLinkDist();            
            if min_dist < hard_dist
                flag = true;
                return;
            end
            if ~isempty(obj.buffer_linklink)
                obj.evaluateLinkLinkDist();
                if min(obj.linklinkDist) < hard_dist
                    flag = true;                    
                end
            end     
        end
    end   
end
