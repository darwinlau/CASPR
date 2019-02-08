% A container class to hold workspace analysis information for a point pose
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    : This class contains the known information obtained
% through workspace analysis.  That is the pose and any metrics/workspace
% conditions that have been evalauted at that point.
classdef PointWorkspaceElement < handle
    properties(SetAccess = protected)
        pose        % The pose for the workspace condition to be evaluated at
        metrics     % A cell array of different metrics (enum and value)
        conditions  % A cell array of different workspace conditions (enum and value)
    end
    
    methods
        % Constructor for the class
        function wp = PointWorkspaceElement(pose,n_metrics,n_constraints)
            wp.pose         = pose;
            wp.metrics      = cell(n_metrics,2);
            wp.conditions   = cell(n_constraints,2);
        end
        
        % A function to add the metric information to the point
        function addMetric(obj,metric_type,metric_value,index)
            obj.metrics{index,1} = metric_type;
            obj.metrics{index,2} = metric_value;
        end
        
        % A function to add a new condition to the point
        function addCondition(obj,condition_type,index)
            obj.conditions{index,1} = condition_type;
            obj.conditions{index,2} = true;
        end
    end    
end