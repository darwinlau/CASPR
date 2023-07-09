% A container class
% Author        : Paul Cheng
% Created       : 2020
% Description    :

classdef ReconfigElement < handle
    properties(SetAccess = protected)
        q                       % Pose of robot
        conditions              % Array of different workspace conditions evaluated
        compTime                % Computational time to compute the ray
        OA_begin
        OA_end
%         reconfigDirVector       % 3x1 direction vector
        reconfigLength          % the length change of the reconfig point
        isInsideObstacle        % if the config is known with collision then it is 1 otherwise 0
        SweptSurfaceDegree
    end
    properties
        FeasibleInterval
        FeasibleOARange
        NeedReconfigure
        ClosestReconfigureRange
    end
    methods
        % Constructor for the class
        function r = ReconfigElement(model, q, conditions, OA_begin, OA_end)
            r.q                    = q;
            r.conditions           = conditions;
            r.OA_begin             = OA_begin;
            r.OA_end               = OA_end;
            r.compTime             = 0;
            r.NeedReconfigure      = zeros(1,model.numCables);
            r.ClosestReconfigureRange = ones(1,model.numCables);
%             r.SweptSurfaceDegree = 
%             for i = 1:model.numCables
%                 reconfigDirVector = (OA_end(:,i) - OA_begin(:,i))./norm(OA_end(:,i) - OA_begin(:,i));
%                 if ~isnan(reconfigDirVector)
%                     r.reconfigDirVector  =   reconfigDirVector;
%                     r.reconfigLength     = norm(OA_end(:,i) - OA_begin(:,i));
%                     break
%                 end
%             end
           
            comp_time = 0;
            for i = 1:model.numCables
                OA_current(:,i) =  model.cableModel.cables{i}.attachments{1}.r_GA ;
            end
            for c_i = 1:size(conditions,2)
%                 if conditions{c_i}.type  == 'INTERFERENCE_CABLE_OBSTACLE'
%                    [condition_intervals{c_i}, ~, comp_time_i] = conditions{c_i}.evaluate(r);
%                 else
% [condition_intervals{c_i}, ~, comp_time_i] = conditions{c_i}.evaluate(r); 
                   [condition_intervals{c_i}, ~, comp_time_i] = conditions{c_i}.evaluate(r); 
%                 end
                comp_time = comp_time + comp_time_i;
            end


            
            for cable_id = 1:model.numCables
                
                den = norm((OA_end(:,cable_id) - OA_begin(:,cable_id)));
                if den == 0
                    r.FeasibleOARange{cable_id}{1} = [OA_end(:,cable_id), OA_begin(:,cable_id)];
                    r.FeasibleInterval{cable_id}{1} = [0,0];
                else                    
                    for c_i = 1:size(conditions,2)
                        interval_c_i = cell2mat(condition_intervals{c_i}{cable_id});

                        if ~isempty(interval_c_i)
                            num = vecnorm((interval_c_i - OA_begin(:,cable_id)));
                            u_curernt = vecnorm(OA_current(:,cable_id) - OA_begin(:,cable_id))/den;
                            if c_i == 1
                                range_a = num/den;                               
                            else
                                range_a = r.IntervalIntersection(range_a,num/den);
                            end
                        else
                            range_a = [];
                        end
                    end
                    range_a = unique(round(range_a,9));
                    column_count = 1;
                    tmp_range_a = [];
                    for a_num = 1:size(range_a,2)
                        if  rem(a_num, 2) == 0
                            tmp_range_a(column_count,2) = range_a(a_num);
                            column_count = column_count +1;
                        else
                        tmp_range_a(column_count,1) = range_a(a_num);
                        end
                    end
                    range_a = tmp_range_a;
                    min_dist = Inf;
                    if ~isempty(range_a)
                        for range_ind = 1:size(range_a,1)
                            r.FeasibleOARange{cable_id}{range_ind} = OA_begin(:,cable_id)  + (OA_end(:,cable_id) - OA_begin(:,cable_id))*range_a(range_ind,:);
                            r.FeasibleInterval{cable_id}{range_ind} = range_a(range_ind,:);                           
                        if any(range_a(range_ind,2) < u_curernt || range_a(range_ind,1) > u_curernt)
                                    r.NeedReconfigure(cable_id) = 1;
                        end
                        end
                    else
                        r.FeasibleOARange{cable_id}{1} = [];
                        r.FeasibleInterval{cable_id}{1} = [];
                    end
                    if r.NeedReconfigure(cable_id)
                    [~,Idx] = min(abs(tmp_range_a - u_curernt),[],1);
                    r.ClosestReconfigureRange(cable_id) = Idx(1);
                        
                    end
                    %                 [val_1,ind_1] = min(length);
                    %                 tmp_interval{cable_id} = condition_intervals{ind_1}{cable_id};
                end
                
            end
            r.FeasibleOARange = r.FeasibleOARange';
            r.FeasibleInterval = r.FeasibleInterval';
            %             r.FeasibleOARange = condition_intervals';
            r.compTime = comp_time;
        end

    end
    methods (Static)
        function out= IntervalIntersection(first,second)
            % Purpose: Range/interval intersection
            %
            % A and B two ranges of closed intervals written
            % as vectors [lowerbound1 upperbound1 lowerbound2 upperbound2]
            % or as matrix [lowerbound1, lowerbound2, lowerboundn;
            %               upperbound1, upperbound2, upperboundn]
            % A and B have to be sorted in ascending order
            %
            % out is the mathematical intersection A n B
            %
            %
            % EXAMPLE USAGE:
            %   >> out=range_intersection([1 3 5 9],[2 9])
            %   	out =  [2 3 5 9]
            %   >> out=range_intersection([40 44 55 58], [42 49 50 52])
            %   	out =  [42 44]
            %
            % Author: Xavier Beudaert <xavier.beudaert@gmail.com>
            % Original: 10-June-2011
            % Major modification and bug fixing 30-May-2012
            % Allocate, as we don't know yet the size, we assume the largest case
            out1(1:(numel(second)+(numel(first)-2)))=0;
            k=1;
            while isempty(first)==0 && isempty(second)==0
                % make sure that first is ahead second
                if first(1)>second(1)
                    temp=second;
                    second=first;
                    first=temp;
                end
                if first(2)<second(1)
                    first=first(3:end);
                    continue;
                elseif first(2)==second(1)
                    out1(k)=second(1);
                    out1(k+1)=second(1);
                    k=k+2;

                    first=first(3:end);
                    continue;
                else
                    if first(2)==second(2)
                        out1(k)=second(1);
                        out1(k+1)=second(2);
                        k=k+2;

                        first=first(3:end);
                        second=second(3:end);

                    elseif first(2)<second(2)
                        out1(k)=second(1);
                        out1(k+1)=first(2);
                        k=k+2;

                        first=first(3:end);
                    else
                        out1(k)=second(1);
                        out1(k+1)=second(2);
                        k=k+2;

                        second=second(3:end);
                    end
                end
            end
            % Remove the tails
            out=out1(1:k-1);
        end

    end
end