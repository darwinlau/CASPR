% A measure of calculating the min distance of all cables
% Author        : Zeqing Zhang
% Created       : 2016
% Description   : Interference metric
classdef MinCableCableDistanceMetric < WorkspaceMetricBase
    properties 
        mindis_mn % save the min distance for all WorkspacePoints
    end
    
    methods
        % Constructor
        function m = MinCableCableDistanceMetric()
            m.metricMin = 0;
            m.metricMax = 0.12; %unit is m
        end
        
        % Evaluate function implementation
        function v = evaluateFunction(obj,dynamics,~) % default of obj is itself
            % pick up a seg. s1 from cable n and the other seg. s2 from cable m (may or may not same)
            % the total munber of calculation for an input model at one worksapce point
            sumcal_model = 0.5 * dynamics.numCables * (1 + dynamics.numCables);
            obj.mindis_mn = zeros(1, sumcal_model);
            % calculate min distanc of these two segments 
            for n = 1:dynamics.numCables  
                % for the optimal purpose, m comes from 1 to n
                for m = 1:1:n
                    % in the same cable, calculate the min distance betweem different seg. of this cable 
                    if m == n
                        % if one cable owns seg. less than 3 , i.e., one cable comprises 2 seg. or 1 seg., the calculation is trivial
                        if dynamics.cableModel.cables{n}.numSegments < 3
                            matsamecable = +inf;
                        else
                            sumcal_samecable = 0.5 * (dynamics.cableModel.cables{n}.numSegments - 1) * (dynamics.cableModel.cables{n}.numSegments - 2);
                            matsamecable = zeros(1, sumcal_samecable);
                            i = 1;
                            % to obtain the abs. location expressed in the frame {F_O} of 2 end points of s1 seg.
                            for s1 = 1:dynamics.cableModel.cables{n}.numSegments % s1 s2 is a index of segment (scalar)
                                s1_p1 = dynamics.cableModel.cables{n}.segments{s1}.attachments{1}.r_OA; % 1 refers to 'from'
                                s1_p2 = dynamics.cableModel.cables{n}.segments{s1}.attachments{2}.r_OA; % 2 refers to 'to'
                                for s2 = 1:s1
                                    % to obtain the abs. location expressed in the frame {F_O} of 2 end points of s2 seg
                                    if s2 ~= s1 && s2 ~= s1 - 1 % the calculation between the same seg. and 2 adjacent seg. is trivial
                                       s2_p1 = dynamics.cableModel.cables{n}.segments{s2}.attachments{1}.r_OA; % 1 refers to 'from'
                                       s2_p2 = dynamics.cableModel.cables{n}.segments{s2}.attachments{2}.r_OA; % 2 refers to 'to'
                                       %by Geometric Method
                                        v1 = s1_p2 - s1_p1;
                                        v2 = s2_p2 - s2_p1;
                                        if all(v1 == 0)
                                            str_a = num2str(s1_p1');
                                            str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
                                            CASPR_log.Warn(str); 
                                            d = obj.d_p2s(s1_p1, [s2_p1, s2_p2]);
                                        elseif all(v2 == 0)
                                            str_a = num2str(s2_p1');
                                            str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
                                            CASPR_log.Warn(str);   
                                            d = obj.d_p2s(s2_p1, [s1_p1, s1_p2]);
                                        else
                                            % non-zero seg. length
                                            v1a = s2_p1 - s1_p1;
                                            s1_vec = [s1_p1, s1_p2];  % s1_vec is a 3X2 vector
                                            s2_vec = [s2_p1, s2_p2];  % s2_vec is a 3X2 vector
                                            g = dot(v1a,cross(v1,v2)); % judge whether 2 segs are coplanar or not
                                            g = round(g, 6); % round off the error to make sure g is integer 1/11
                                                             % add 6 to round off the real numerical error 30/12
                                            if g == 0 % coplanar 
                                                d = d_uni(obj, s1_vec, s2_vec); % obj.d_uni(s1_vec, s2_vec)
                                            else
                                                d = d_non_F2(obj, s1_vec, s2_vec); % with judging 
                                            end
                                        end
                                        % store the min distance of seg. s1 and seg. s2 
                                        % among the same cable n (m == n)
                                        matsamecable(i) = d;
                                        i = i + 1;
                                    end
                                end
                            end
                        end
                        % find the minimum value of min distance of the same cable of the input
                        % model, which means the minimum value of min distance of different segments of one cable
                        diag = 0.5 * n * (n + 1);
                        obj.mindis_mn(diag) = min(min(matsamecable));
                    % two cables selected in advance are different
                    elseif m ~= n 
                        matdiffcable = zeros(dynamics.cableModel.cables{n}.numSegments, dynamics.cableModel.cables{m}.numSegments);
                        for s1 = 1:dynamics.cableModel.cables{n}.numSegments %s1 s2 is a index of segment (scalar)
                            s1_p1 = dynamics.cableModel.cables{n}.segments{s1}.attachments{1}.r_OA; % 1 refers to 'from'
                            s1_p2 = dynamics.cableModel.cables{n}.segments{s1}.attachments{2}.r_OA; % 2 refers to 'to'
                            for s2 = 1:dynamics.cableModel.cables{m}.numSegments
                                s2_p1 = dynamics.cableModel.cables{m}.segments{s2}.attachments{1}.r_OA;
                                s2_p2 = dynamics.cableModel.cables{m}.segments{s2}.attachments{2}.r_OA;
                                % by Geometric Method
                                v1 = s1_p2 - s1_p1; % from A to B
                                v2 = s2_p2 - s2_p1;
                                % v1(v2) and v1_temp(v2_temp) are indentical
                                % v1_temp = dynamics.cableModel.cables{n}.segments{s1}.segmentVector
                                % v2_temp = dynamics.cableModel.cables{m}.segments{s2}.segmentVector
                                % warning in case of 0 cable/seg. length [30/12]
                                if all(v1 == 0)
                                    str_a = num2str(s1_p1');
                                    str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
                                    CASPR_log.Warn(str); 
                                    d = obj.d_p2s(s1_p1, [s2_p1, s2_p2]);
                                elseif all(v2 == 0)
                                    str_a = num2str(s2_p1');
                                    str = strcat('The length of segment starting from', ' [', str_a, ']', ' is zero!');
                                    CASPR_log.Warn(str);   
                                    d = obj.d_p2s(s2_p1, [s1_p1, s1_p2]);
                                else
                                    % non-zero seg. length
                                    v1a = s2_p1 - s1_p1;
                                    s1_vec = [s1_p1, s1_p2];  % s1_vec is a 3X2 vector
                                    s2_vec = [s2_p1, s2_p2];  % s2_vec is a 3X2 vector
                                    g = dot(v1a,cross(v1,v2)); % judge whether 2 segs are coplanar or not
                                    g = round(g, 6); % round off the error to make sure g is integer 1/11
                                                     % add 6 to round off the real numerical error 30/12
                                    if g == 0 % coplanar 
                                        d = d_uni(obj, s1_vec, s2_vec); % obj.d_uni(s1_vec, s2_vec)
                                    else
                                        d = d_non_F2(obj, s1_vec, s2_vec); % with judging 
                                    end
                                end
                                % store the min distance of seg. s1 and seg. s2 
                                % among different cable n and cable m
                                matdiffcable(s1, s2) = d;                               
                            end
                        end
                        % find the minimum value of min distance of two different cables
                        % among all cables of the input model
                        belowdiag = 0.5 * n * (n - 1) + m;
                        obj.mindis_mn(belowdiag) = min(min(matdiffcable));
                    end
                end
             end 
            % find the min distance among all cables of the input model
            v = min(min(obj.mindis_mn)); 
            % obj.mindis_mn in the form of a lower triangular matrix
            temp = zeros(dynamics.numCables, dynamics.numCables);
            count = 1;
            for i = 1: dynamics.numCables
                for j = 1: i
                temp(i, j) = obj.mindis_mn(count);
                count = count + 1;
                end
            end
            obj.mindis_mn = temp;
        end
        
        function d = d_p2s(~, p, s)
            %To calculate the distance from a point to a segment
            % p is 3X1 vector containing point coordinate
            % s is 3X2 matrix of a segment of 2(two) endpoints coordinate
            s_p1 = s(:,1); 
            s_p2 = s(:,2); 
            p1p2 = s_p2 - s_p1;
            p2p1 = s_p1 - s_p2;
            p1p = p - s_p1;
            p2p = p - s_p2;
            d1 = sum(p1p2.*p1p);  % dot(p1p2,p1p)
            d2 = sum(p2p1.*p2p);  % dot(p2p1,p2p)
            if d1 < 0
                d = norm(p1p);
            elseif d2 < 0
                d = norm(p2p);
            else
                num = norm(cross(p1p,p2p));
                den = norm(p1p2);
                d = num/den;
            end    
        end
        
        
        
        function d = d_non_F2(obj, s1, s2)
            %To calculate the shortest distance of two non-nuiplanar segments by
            %[Judging 2 conditions]
            %s1,s2 mean two seg1 and seg2 [non-uniplanar]
            s1_p1 = s1(:,1); 
            s1_p2 = s1(:,2); 
            s2_p1 = s2(:,1); 
            s2_p2 = s2(:,2); 
            v1 = s1_p2 - s1_p1;
            v1a = s2_p1 - s1_p1;
            v2 = s2_p2 - s2_p1;

            %calculate the length of common perpendicular c
            c_num = norm(dot(v1a,cross(v1,v2))); % like abs. value of dot product
            c_den = norm(cross(v1,v2));
            c = c_num/c_den;

            %calcualte the shortest dis. of seg1 and seg2'[projection] e
            lc_unit = cross(v1,v2)/norm(cross(v1,v2));

            %judge which is the right direction of projection 1/11
            lc1 = c .* (lc_unit);
            lc2 = c .* (-lc_unit);
            s2_p1_pri1 = s2_p1 + lc1; 
            temp = s2_p1_pri1 - s1_p1;
            g = dot(temp,cross(v1,v2));
            g = round(g, 6); % round off the error to make sure g is integer 1/11
                             % add 6 to round off the real numerical error 30/12
            if g == 0
                lc = lc1;
            else
                lc = lc2;
            end
            s2_p1_pri = s2_p1 + lc;
            s2_p2_pri = s2_p2 + lc;

            % using the projection of the seg 2 to judge whether crossing or not later
            v1 = s1_p2 - s1_p1;
            v1a = s2_p1_pri - s1_p1;
            v1b = s2_p2_pri - s1_p1;
            v2 = s2_p2_pri - s2_p1_pri;
            v2a = s1_p1 - s2_p1_pri;
            v2b = s1_p2 - s2_p1_pri;
            c1a = cross(v1, v1a);
            c1b = cross(v1, v1b);
            c2a = cross(v2, v2a);
            c2b = cross(v2, v2b);
            c1 = sum(c1a.*c1b);
            c2 = sum(c2a.*c2b);

            % judge crossing or not
            if c1<0 && c2<0
                d = c;
            else
                d1 = obj.d_p2s(s1_p1, s2);
                d2 = obj.d_p2s(s1_p2, s2);
                d3 = obj.d_p2s(s2_p1, s1);
                d4 = obj.d_p2s(s2_p2, s1);
                d = min([d1, d2, d3, d4]);
            end
        end
        
        
        function d = d_uni(obj, s1_vec, s2_vec)
            %To calculate the shortest distance of two nuiplanar segments
            %s1,s2 mean two seg1 and seg2[uniplanar]
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%[30/12]%%%%%%
            % Judge whether 4 end pts intersect or not.
            % In one case there were s1_p1 = [1, 1, 0] given by xml file and s2_p2 = [1.0000, 1.0000, 0] 
            % computed by IK, CPU thought they are not the same. 
            % So round off the numerical errors by following:
            s1_vec =round(s1_vec, 6);
            s2_vec =round(s2_vec, 6);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            s1_p1 = s1_vec(:,1); 
            s1_p2 = s1_vec(:,2); 
            s2_p1 = s2_vec(:,1); 
            s2_p2 = s2_vec(:,2); 
            if all(s1_p1 == s2_p1) || all(s1_p1 == s2_p2) || all(s1_p2 == s2_p1) || all(s1_p2 == s2_p2)
                d = 0;
            else
                v1 = s1_p2 - s1_p1;
                v1a = s2_p1 - s1_p1;
                v1b = s2_p2 - s1_p1;
                v2 = s2_p2 - s2_p1;
                v2a = s1_p1 - s2_p1;
                v2b = s1_p2 - s2_p1;
                c1a = cross(v1, v1a);
                c1b = cross(v1, v1b);
                c2a = cross(v2, v2a);
                c2b = cross(v2, v2b);
                c1 = sum(c1a.*c1b);
                c2 = sum(c2a.*c2b);
                if c1<0 && c2<0
                    d = 0;
                else
                    d1 = obj.d_p2s(s1_p1, s2_vec);
                    d2 = obj.d_p2s(s1_p2, s2_vec);
                    d3 = obj.d_p2s(s2_p1, s1_vec);
                    d4 = obj.d_p2s(s2_p2, s1_vec);
                    d = min([d1, d2, d3, d4]);
                end
            end
        end
    end
end