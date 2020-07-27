% A container class to hold workspace analysis information for a point pose
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    : This class contains the known information obtained
% through workspace analysis.  That is the pose and any metrics/workspace
% conditions that have been evalauted at that point.
classdef PointWorkspaceElement < handle
    properties(SetAccess = protected)
        pose                    % The pose for the workspace condition to be evaluated at
        metrics = {};           % A cell array of different metrics (metric object, value)
        conditions = {};        % A cell array of different workspace conditions (conditon object)
        conditionsAll = {};     % A cell array of all conditions evaluated
        conditionsIndices = []; % Array of indices of the conditions from conditionsAll (and same as PointWorkspace)
        compTime = 0;           % Computational time of the workspace element                
    end
    
    methods
        % Constructor for the class
        function wp = PointWorkspaceElement(q, modelObj, conditions, metrics)
            CASPR_log.Assert(~(isempty(conditions) && isempty(metrics)), 'At least one workspace condition or metric is required');
            wp.pose             = q;
            wp.conditionsAll    = conditions;
            
            t_comp = tic;
            % Checks if the modelObj is already this q value (and hence
            % updated)
            if (~isequal(q, modelObj.q))
                modelObj.update(q, zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1),zeros(modelObj.numDofs,1));
            end
            
            wp.evaluateMetrics(modelObj, metrics);
            % Pass the evaluated metrics into the conditions to help
            wp.evaluateConditions(modelObj, conditions, wp.metrics);
            wp.compTime = toc(t_comp);
        end
        
        % Evaluate all metrics and store values
        function evaluateMetrics(obj, model, metrics)
            obj.metrics = PointWorkspaceElement.EvaluateMetrics(model, metrics);
        end
        
        function evaluateConditions(obj, model, conditions, evaluated_metrics)
            [condition_results] = PointWorkspaceElement.EvaluateConditions(model, conditions, evaluated_metrics);
            if (~isempty(condition_results))
                n_conditions = sum(cell2mat(condition_results(:, 2)));
                obj.conditions = cell(n_conditions, 1);
                c_ind = 1;
                for ind = 1:length(conditions)
                    if (condition_results{ind, 2})
                        obj.conditions{c_ind, 1} = condition_results{ind, 1};
                        obj.conditionsIndices(c_ind, 1) = ind;
                        c_ind = c_ind + 1;
                    end
                end
            end
        end
    end    
    
    methods (Static)
        % Function evaluates a cell array of metrics
        % Input: model object
        % Input: cell array of metrics
        % Output: An array of metric types (WorkspaceMetricType)
        % Output: An array of metric values
        function [results] = EvaluateMetrics(model, metrics)
            results = cell(length(metrics), 2);
            for i = 1:length(metrics)
                results{i, 1} = metrics{i};
                results{i, 2} = metrics{i}.evaluate(model);
            end
        end
        
        % Function evaluates a cell array of conditions
        % Input: model object
        % Input: cell array of conditions
        % Output: An array of condition types (WorkspaceConditionType)
        % Output: An array of condition values
        function [results] = EvaluateConditions(model, conditions, evaluated_metrics)
            results = cell(length(conditions), 2);
            for i = 1:length(conditions)
                results{i, 1} = conditions{i};
                results{i, 2} = conditions{i}.evaluate(model, evaluated_metrics);
            end
        end
    end
end