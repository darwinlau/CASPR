% A container class to hold workspace analysis information
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    : This class contains the known information obtained
% through workspace analysis.  That is the pose and any metrics/workspace
% conditions that have been evalauted at that point.
classdef WorkspacePoint < handle
    properties(SetAccess = protected)
        pose        % The pose for the workspace condition to be evaluated at
        metrics     % A cell array of different metrics (enum and value)
        conditions  % A cell array of different workspace conditions (enum and value)
    end
    
    methods
        function wp = WorkspacePoint(pose)
            wp.pose         = pose;
            wp.metrics      = cell(0);
            wp.conditions   = cell(0);
        end
        
        function addMetric(obj,metric_type,metric_value)
            if(~isempty(obj.metrics))
                % Test that this is a new metric
                if(sum(metric_type == obj.metrics(:,1))==0)    
                    s_l = size(obj.metrics,1)+1;
                    obj.metrics{s_l,1} = metric_type;
                    obj.metrics{s_l,2} = metric_value;
                end
            else
                obj.metrics{1,1} = metric_type;
                obj.metrics{1,2} = metric_value;
            end
        end
        
        function addCondition(obj,condition_type,condition_value)
            if(~isempty(obj.conditions))
                % Test that this is a new metric
                if(sum(condition_type == obj.conditions(:,1))==0)    
                    s_l = size(obj.conditions,1)+1;
                    obj.conditions{s_l,1} = condition_type;
                    obj.conditions{s_l,2} = condition_value;
                end
            else
                obj.conditions{1,1} = condition_type;
                obj.conditions{1,2} = condition_value;
            end
        end
    end    
end