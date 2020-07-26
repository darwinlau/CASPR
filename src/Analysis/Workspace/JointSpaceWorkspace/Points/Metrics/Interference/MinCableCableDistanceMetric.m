% A measure of calculating the min distance of all cables
% Author        : Zeqing Zhang
% Created       : 2016
% Description   : Interference metric
classdef MinCableCableDistanceMetric < WorkspaceMetricBase    
    % Constants that needs to be defined from parent
    properties (Constant)
        type = WorkspaceMetricType.MIN_CABLE_CABLE_DISTANCE;
        metricMin = 0;
        metricMax = Inf;
    end
    
    properties 
        mindis_mn % save the min distance for all WorkspacePoints
    end
    
    methods
        % Constructor
        function m = MinCableCableDistanceMetric()
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(obj,dynamics) % default of obj is itself
%             % pick up a seg. s1 from cable n and the other seg. s2 from cable m (may or may not same)
%             % the total munber of calculation for an input model at one worksapce point
%             sumcal_model = 0.5 * dynamics.numCables * (1 + dynamics.numCables);
%             obj.mindis_mn = zeros(1, sumcal_model);
%             
            v = Inf;

            % This method uses the compiled r_OAs matrix to check for
            % interference
            
            % Go through every cables and segments
            cable_combinations = nchoosek(1:size(dynamics.cableModel.r_OAs, 2), 2);
            num_cable_combs = size(cable_combinations, 1);
            
            for k = 1:num_cable_combs
                i = cable_combinations(k, 1);
                j = cable_combinations(k, 2);
                
                A_i = dynamics.cableModel.r_OAs(1:3, i);
                B_i = dynamics.cableModel.r_OAs(4:6, i);
                A_j = dynamics.cableModel.r_OAs(1:3, j);
                B_j = dynamics.cableModel.r_OAs(4:6, j);
                
                AB_i = B_i - A_i; % from A to B
                AB_j = B_j - A_j;
                
                % warning in case of 0 cable/seg. length [30/12]
                if all(AB_i == 0)
                    CASPR_log.Warn(sprintf('The length of segment starting from [%s] is zero!', num2str(A_i')));
                    d = obj.DistancePointAndSegment(A_i, [A_j, B_j]);
                elseif all(AB_j == 0)
                    CASPR_log.Warn(sprintf('The length of segment starting from [%s] is zero!', num2str(A_j')));
                    CASPR_log.Warn(str);
                    d = obj.DistancePointAndSegment(A_j, [A_i, B_i]);
                else
                    % non-zero seg. length
                    v1a = A_j - A_i;
                    s1_vec = [A_i, B_i];  % s1_vec is a 3X2 vector
                    s2_vec = [A_j, B_j];  % s2_vec is a 3X2 vector
                    g = dot(v1a,cross(AB_i,AB_j)); % judge whether 2 segs are coplanar or not
                    g = round(g, 6); % round off the error to make sure g is integer 1/11
                    % add 6 to round off the real numerical error 30/12
                    if g == 0 % coplanar
                        d = obj.DistanceSegmentsUniplanar(s1_vec, s2_vec);
                    else
                        d = obj.DistanceSegmentsNonUniplanar(s1_vec, s2_vec); % with judging
                    end
                end
                v = min(v, d);
            end
%             
%             % calculate min distance of these two segments 
%             for ind_i = 1:dynamics.numCables  
%                 for ind_j = 1:ind_i
%                     % in the same cable, calculate the min distance betweem different seg. of this cable 
%                     if ind_j == ind_i
%                         % if one cable owns seg. less than 3 , i.e., one cable comprises 2 seg. or 1 seg., the calculation is trivial
%                         if dynamics.cableModel.cables{ind_i}.numSegments < 3
%                             matsamecable = +inf;
%                         else
%                             sumcal_samecable = 0.5 * (dynamics.cableModel.cables{ind_i}.numSegments - 1) * (dynamics.cableModel.cables{ind_i}.numSegments - 2);
%                             matsamecable = zeros(1, sumcal_samecable);
%                             i = 1;
%                             % to obtain the abs. location expressed in the frame {F_O} of 2 end points of s1 seg.
%                             for s1 = 1:dynamics.cableModel.cables{ind_i}.numSegments % s1 s2 is a index of segment (scalar)
%                                 A_i = dynamics.cableModel.cables{ind_i}.segments{s1}.attachments{1}.r_OA; % 1 refers to 'from'
%                                 B_i = dynamics.cableModel.cables{ind_i}.segments{s1}.attachments{2}.r_OA; % 2 refers to 'to'
%                                 for s2 = 1:s1
%                                     % to obtain the abs. location expressed in the frame {F_O} of 2 end points of s2 seg
%                                     if s2 ~= s1 && s2 ~= s1 - 1 % the calculation between the same seg. and 2 adjacent seg. is trivial
%                                        A_j = dynamics.cableModel.cables{ind_i}.segments{s2}.attachments{1}.r_OA; % 1 refers to 'from'
%                                        B_j = dynamics.cableModel.cables{ind_i}.segments{s2}.attachments{2}.r_OA; % 2 refers to 'to'
%                                        %by Geometric Method
%                                         AB_i = B_i - A_i;
%                                         AB_j = B_j - A_j;
%                                         if all(AB_i == 0)
%                                             str_a = num2str(A_i');
%                                             str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
%                                             CASPR_log.Warn(str); 
%                                             d = obj.DistancePointAndSegment(A_i, [A_j, B_j]);
%                                         elseif all(AB_j == 0)
%                                             str_a = num2str(A_j');
%                                             str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
%                                             CASPR_log.Warn(str);   
%                                             d = obj.DistancePointAndSegment(A_j, [A_i, B_i]);
%                                         else
%                                             % non-zero seg. length
%                                             v1a = A_j - A_i;
%                                             s1_vec = [A_i, B_i];  % s1_vec is a 3X2 vector
%                                             s2_vec = [A_j, B_j];  % s2_vec is a 3X2 vector
%                                             g = dot(v1a,cross(AB_i,AB_j)); % judge whether 2 segs are coplanar or not
%                                             g = round(g, 6); % round off the error to make sure g is integer 1/11
%                                                              % add 6 to round off the real numerical error 30/12
%                                             if g == 0 % coplanar 
%                                                 d = obj.DistanceSegmentsUniplanar(s1_vec, s2_vec); % obj.DistanceSegmentsUniplanar(s1_vec, s2_vec)
%                                             else
%                                                 d = obj.DistanceSegmentsNonUniplanar(s1_vec, s2_vec); % with judging 
%                                             end
%                                         end
%                                         % store the min distance of seg. s1 and seg. s2 
%                                         % among the same cable n (m == n)
%                                         matsamecable(i) = d;
%                                         i = i + 1;
%                                     end
%                                 end
%                             end
%                         end
%                         % find the minimum value of min distance of the same cable of the input
%                         % model, which means the minimum value of min distance of different segments of one cable
%                         diag = 0.5 * ind_i * (ind_i + 1);
%                         obj.mindis_mn(diag) = min(min(matsamecable));
%                     % two cables selected in advance are different
%                     elseif ind_j ~= ind_i 
%                         matdiffcable = zeros(dynamics.cableModel.cables{ind_i}.numSegments, dynamics.cableModel.cables{ind_j}.numSegments);
%                         for s1 = 1:dynamics.cableModel.cables{ind_i}.numSegments %s1 s2 is a index of segment (scalar)
%                             A_i = dynamics.cableModel.cables{ind_i}.segments{s1}.attachments{1}.r_OA; % 1 refers to 'from'
%                             B_i = dynamics.cableModel.cables{ind_i}.segments{s1}.attachments{2}.r_OA; % 2 refers to 'to'
%                             for s2 = 1:dynamics.cableModel.cables{ind_j}.numSegments
%                                 A_j = dynamics.cableModel.cables{ind_j}.segments{s2}.attachments{1}.r_OA;
%                                 B_j = dynamics.cableModel.cables{ind_j}.segments{s2}.attachments{2}.r_OA;
%                                 % by Geometric Method
%                                 AB_i = B_i - A_i; % from A to B
%                                 AB_j = B_j - A_j;
%                                 % v1(v2) and v1_temp(v2_temp) are indentical
%                                 % v1_temp = dynamics.cableModel.cables{n}.segments{s1}.segmentVector
%                                 % v2_temp = dynamics.cableModel.cables{m}.segments{s2}.segmentVector
%                                 % warning in case of 0 cable/seg. length [30/12]
%                                 if all(AB_i == 0)
%                                     str_a = num2str(A_i');
%                                     str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
%                                     CASPR_log.Warn(str); 
%                                     d = obj.DistancePointAndSegment(A_i, [A_j, B_j]);
%                                 elseif all(AB_j == 0)
%                                     str_a = num2str(A_j');
%                                     str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
%                                     CASPR_log.Warn(str);   
%                                     d = obj.DistancePointAndSegment(A_j, [A_i, B_i]);
%                                 else
%                                     % non-zero seg. length
%                                     v1a = A_j - A_i;
%                                     s1_vec = [A_i, B_i];  % s1_vec is a 3X2 vector
%                                     s2_vec = [A_j, B_j];  % s2_vec is a 3X2 vector
%                                     g = dot(v1a,cross(AB_i,AB_j)); % judge whether 2 segs are coplanar or not
%                                     g = round(g, 6); % round off the error to make sure g is integer 1/11
%                                                      % add 6 to round off the real numerical error 30/12
%                                     if g == 0 % coplanar 
%                                         d = obj.DistanceSegmentsUniplanar(s1_vec, s2_vec);
%                                     else
%                                         d = obj.DistanceSegmentsNonUniplanar(s1_vec, s2_vec); % with judging 
%                                     end
%                                 end
%                                 % store the min distance of seg. s1 and seg. s2 
%                                 % among different cable n and cable m
%                                 matdiffcable(s1, s2) = d;                               
%                             end
%                         end
%                         % find the minimum value of min distance of two different cables
%                         % among all cables of the input model
%                         belowdiag = 0.5 * ind_i * (ind_i - 1) + ind_j;
%                         obj.mindis_mn(belowdiag) = min(min(matdiffcable));
%                     end
%                 end
%             end 
%             % find the min distance among all cables of the input model
%             v = min(min(obj.mindis_mn)); 
%             % obj.mindis_mn in the form of a lower triangular matrix
%             temp = zeros(dynamics.numCables, dynamics.numCables);
%             count = 1;
%             for i = 1: dynamics.numCables
%                 for j = 1: i
%                 temp(i, j) = obj.mindis_mn(count);
%                 count = count + 1;
%                 end
%             end
%             obj.mindis_mn = temp;
        end
    end
    
    methods (Static)
        function d = DistancePointAndSegment(P, seg)
            %To calculate the distance from a point to a segment
            % p is 3X1 vector containing point coordinate
            % s is 3X2 matrix of a segment of 2 (two) endpoints coordinate
            A1 = seg(:,1); 
            A2 = seg(:,2); 
            A1A2 = A2 - A1;
            A2A1 = A1 - A2;
            A1P = P - A1;
            A2P = P - A2;
            d1 = sum(A1A2.*A1P);  % dot(p1p2,p1p)
            d2 = sum(A2A1.*A2P);  % dot(p2p1,p2p)
            if d1 < 0
                d = norm(A1P);
            elseif d2 < 0
                d = norm(A2P);
            else
                num = norm(cross(A1P,A2P));
                den = norm(A1A2);
                d = num/den;
            end    
        end
        
        
        function d = DistanceSegmentsNonUniplanar(segPoints_1, segPoints_2)
            %To calculate the shortest distance of two non-nuiplanar segments by
            %[Judging 2 conditions]
            %s1,s2 mean two seg1 and seg2 [non-uniplanar]
            A1 = segPoints_1(:,1); 
            B1 = segPoints_1(:,2); 
            A2 = segPoints_2(:,1); 
            B2 = segPoints_2(:,2); 
            A1B1 = B1 - A1;
            v1a = A2 - A1;
            v2 = B2 - A2;

            %calculate the length of common perpendicular c
            c_num = norm(dot(v1a,cross(A1B1,v2))); % like abs. value of dot product
            c_den = norm(cross(A1B1,v2));
            c = c_num/c_den;

            %calcualte the shortest dis. of seg1 and seg2'[projection] e
            lc_unit = cross(A1B1,v2)/norm(cross(A1B1,v2));

            %judge which is the right direction of projection 1/11
            lc1 = c .* (lc_unit);
            lc2 = c .* (-lc_unit);
            s2_p1_pri1 = A2 + lc1; 
            temp = s2_p1_pri1 - A1;
            g = dot(temp,cross(A1B1,v2));
            g = round(g, 6); % round off the error to make sure g is integer 1/11
                             % add 6 to round off the real numerical error 30/12
            if g == 0
                lc = lc1;
            else
                lc = lc2;
            end
            s2_p1_pri = A2 + lc;
            s2_p2_pri = B2 + lc;

            % using the projection of the seg 2 to judge whether crossing or not later
            A1B1 = B1 - A1;
            v1a = s2_p1_pri - A1;
            v1b = s2_p2_pri - A1;
            v2 = s2_p2_pri - s2_p1_pri;
            v2a = A1 - s2_p1_pri;
            v2b = B1 - s2_p1_pri;
            c1a = cross(A1B1, v1a);
            c1b = cross(A1B1, v1b);
            c2a = cross(v2, v2a);
            c2b = cross(v2, v2b);
            c1 = sum(c1a.*c1b);
            c2 = sum(c2a.*c2b);

            % judge crossing or not
            if c1<0 && c2<0
                d = c;
            else
                d1 = MinCableCableDistanceMetric.DistancePointAndSegment(A1, segPoints_2);
                d2 = MinCableCableDistanceMetric.DistancePointAndSegment(B1, segPoints_2);
                d3 = MinCableCableDistanceMetric.DistancePointAndSegment(A2, segPoints_1);
                d4 = MinCableCableDistanceMetric.DistancePointAndSegment(B2, segPoints_1);
                d = min([d1, d2, d3, d4]);
            end
        end
        
        function d = DistanceSegmentsUniplanar(segPoints_1, segPoints_2)
            %To calculate the shortest distance of two uniplanar segments
            %s1,s2 mean two seg1 and seg2[uniplanar]
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%[30/12]%%%%%%
            % Judge whether 4 end pts intersect or not.
            % In one case there were s1_p1 = [1, 1, 0] given by xml file and s2_p2 = [1.0000, 1.0000, 0] 
            % computed by IK, CPU thought they are not the same. 
            % So round off the numerical errors by following:
            segPoints_1 = round(segPoints_1, 6);
            segPoints_2 = round(segPoints_2, 6);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            A1 = segPoints_1(:,1); 
            B1 = segPoints_1(:,2); 
            A2 = segPoints_2(:,1); 
            B2 = segPoints_2(:,2); 
            if all(A1 == A2) || all(A1 == B2) || all(B1 == A2) || all(B1 == B2)
                d = 0;
            else
                A1B1 = B1 - A1;
                A1A2 = A2 - A1;
                A1B2 = B2 - A1;
                A2B2 = B2 - A2;
                A2A1 = A1 - A2;
                A2B1 = B1 - A2;
                c1a = cross(A1B1, A1A2);
                c1b = cross(A1B1, A1B2);
                c2a = cross(A2B2, A2A1);
                c2b = cross(A2B2, A2B1);
                c1 = sum(c1a.*c1b);
                c2 = sum(c2a.*c2b);
                if c1 < 0 && c2 < 0
                    d = 0;
                else
                    d1 = MinCableCableDistanceMetric.DistancePointAndSegment(A1, segPoints_2);
                    d2 = MinCableCableDistanceMetric.DistancePointAndSegment(B1, segPoints_2);
                    d3 = MinCableCableDistanceMetric.DistancePointAndSegment(A2, segPoints_1);
                    d4 = MinCableCableDistanceMetric.DistancePointAndSegment(B2, segPoints_1);
                    d = min([d1, d2, d3, d4]);
                end
            end
        end
    end
end