% A container class to hold workspace analysis information for ray based
% generation
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    : This class contains the known information obtained
% through workspace analysis.  That is the pose and any metrics/workspace
% conditions that have been evalauted at that ray.
classdef WorkspaceRay < handle
    properties(SetAccess = protected)
        fixed_variables         % The pose for the workspace condition to be evaluated at
        metrics                 % A cell array of different metrics (enum and value)
        conditions              % A cell array of different workspace conditions (enum and intervals)
        free_variable_index     % The index that is left free
        free_variable_range     % The range of values that the free varaible takes (1st element should be min and 2nd element max)
    end
    
    methods
        % Constructor for the class
        function wp = WorkspaceRay(fixed_variables,n_metrics,n_constraints,free_variable_index,free_variable_range)
            wp.fixed_variables      =   fixed_variables;
            wp.metrics              =   cell(n_metrics,2);
            wp.conditions           =   cell(n_constraints,2);
            wp.free_variable_index  =   free_variable_index;
            wp.free_variable_range  =   free_variable_range;
        end
        
        % A function to add the metric information to the point
        function addMetric(obj,metric_type,metric_value,index)
            obj.metrics{index,1} = metric_type;
            obj.metrics{index,2} = metric_value;
        end
        
        % A function to add a new condition to the point
        function addCondition(obj,condition_type,intervals,index)
            obj.conditions{index,1} = condition_type;
            obj.conditions{index,2} = intervals;
        end
    end    
end