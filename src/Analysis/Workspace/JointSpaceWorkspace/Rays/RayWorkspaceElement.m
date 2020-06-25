% A container class to hold workspace analysis information for a single
% workspace ray. NOTE: This class assumes that the ray is
% generated about one of the principle axes.
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    : This class contains the known information obtained
% through workspace analysis.  That is the pose and any metrics/workspace
% conditions that have been evalauted at that ray.
classdef RayWorkspaceElement < handle
    properties(SetAccess = protected)
        fixed_variables         % The pose for the workspace condition to be evaluated at
%         metrics                 % A cell array of different metrics (enum and value)
        conditions              % Array of different workspace conditions evaluated
        free_variable_index     % The index that is left free
        free_variable_range
        numDofs                 % The number of degrees of freedom associated with this ray
        interval                % The workspace interval  
    end
    properties (Hidden)
    end
    methods
        % Constructor for the class
        function wp = RayWorkspaceElement(model, fixed_variables, conditions, free_variable_index, free_variable_range)
            wp.fixed_variables      =   fixed_variables;
            wp.free_variable_index  =   free_variable_index;
            wp.free_variable_range  =   free_variable_range;
            wp.numDofs              =   model.numDofs;
            wp.conditions           =   conditions;
            interval_combined       = free_variable_range;
            
            % For each condition
            for c_i = 1:size(conditions,2)
                
                % New condition
                condition_intervals = conditions{c_i}.evaluate(model, wp);
                
                if(~isempty(condition_intervals))
                    % There may be multiple intervals
                    temp_intervals = [];
                    for i = 1:size(condition_intervals, 1)
                        for j = 1:size(interval_combined, 1)
                            lower = max(condition_intervals(i, 1), interval_combined(j, 1));
                            higher = min(condition_intervals(i, 2), interval_combined(j, 2));
                            if (lower <= higher)
                                temp_intervals = [temp_intervals; lower, higher];
                            end
                        end
                    end
                    interval_combined = temp_intervals;
                else
                    interval_combined = [];
                end
                
                if (isempty(interval_combined))
                    break;
                end
            end
            wp.interval = interval_combined;
        end
    end
end