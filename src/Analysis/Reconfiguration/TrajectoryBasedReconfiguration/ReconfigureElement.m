classdef ReconfigureElement < handle
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Variable_Range
        Reconfig_Cable_ID
        Unknown_var
        Unknown_length
        Element_Count
        Info
        NumVarInPath
        SweptSurfaceDegree        
       
        
        FeasibleInterval
        FeasibleOARange
        NeedReconfigure
        ClosestReconfigureRange
    end
    
%     properties
%         SweptSurfaceDegree
%     end

    methods
        function obj = ReconfigureElement(reconfig_cable_ind,OA_path,reconfig_var_range,path2varIndex,varargin)
%             if numel(reconfig_cable_ind) ~= size(OA_path,2)
%                 CASPR_log.Error('Number of reconfigurable cable does not match');
%             elseif numel(reconfig_cable_ind) ~= size(reconfig_var_range,2)
%                 CASPR_log.Error('Number of reconfigurable range does not match');
%             end
            
            for i = 1:size(path2varIndex,1)
                obj.NumVarInPath(i,:) = numel(path2varIndex{i,2});
            end
            obj.Reconfig_Cable_ID = reconfig_cable_ind;
            obj.Variable_Range = reconfig_var_range;
%             obj.OA_path = OA_path;
            obj.Element_Count = numel(reconfig_cable_ind) ;
            
            
            if size(reconfig_var_range,2) == 1                
            obj.Unknown_var = sym('u',1,'real');
            obj.Unknown_length = sym('a',1,'real');
            else                
            obj.Unknown_var = sym('u%d',[size(reconfig_var_range,2),1],'real');
            obj.Unknown_length = sym('a%d',[numel(reconfig_cable_ind),1],'real');
            end
            for i = 1:numel(reconfig_cable_ind)
                obj.Info(i).Path = OA_path{i};
                obj.Info(i).CableID = path2varIndex{i,1};
                obj.Info(i).VariableID = path2varIndex{i,2};
                obj.Info(i).Variable = obj.Unknown_var(obj.Info(i).VariableID)';
                obj.Info(i).VariableRange = reconfig_var_range(:,obj.Info(i).VariableID);         
                obj.SweptSurfaceDegree(i) = 0; 
            end
            
            
            if ~isempty(varargin)
                for i = 1:2:size(varargin,2)
                    if strcmpi(varargin{i},'SweptSurfaceDegree')
                        obj.SweptSurfaceDegree = varargin{i+1};                    
                    end
                end
            end
            
        end
        
        
        function interference_range = ReconfigurableRangeEvaluation(obj, model, q, conditions)
            for i = 1:model.numCables
                OA_current(:,i) =  model.cableModel.cables{i}.attachments{1}.r_GA ;
            end
             
            comp_time = 0;
            for c_i = 1:size(conditions,2)
                   [condition_intervals{c_i}, ~, comp_time_i] = conditions{c_i}.evaluate(obj); 
                comp_time = comp_time + comp_time_i;
            end
            
            %%% need to handle multiple condition later
                FeasibleInterval = condition_intervals{1};
            %%%
            
            for CSI = 1:model.numCables
                ReconfigCableID = find(ismember(obj.Reconfig_Cable_ID,CSI));
                if ~isempty(ReconfigCableID)
                    range = obj.Info(ReconfigCableID).VariableRange;
                    OA_i_0 = obj.Info(ReconfigCableID).Path(range(1));
                    OA_i_1 = obj.Info(ReconfigCableID).Path(range(2));
                    
                    u_current = interp1([0,1],range,norm(OA_current(:,CSI)-OA_i_0)/norm(OA_i_1-OA_i_0));
                    
                    c_range = FeasibleInterval{CSI};
                    dist = abs((c_range - u_current));
                    [~,closestIndex] = min(dist(:));
                    [row, col] = ind2sub( size(c_range), closestIndex );
                    obj.ClosestReconfigureRange(CSI) = row;
                    interference_range(CSI,:) = c_range(row,:);
                    if u_current < c_range(row,1) || u_current > c_range(row,2)
                        obj.NeedReconfigure(CSI) = 1;
                    else
                        obj.NeedReconfigure(CSI) = 0;
                    end
                    
                    
                else
                    interference_range(CSI,:) = [1,1];
                    obj.ClosestReconfigureRange(CSI) = 1;
                    obj.NeedReconfigure(CSI) = 0;
                end
            end
            
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