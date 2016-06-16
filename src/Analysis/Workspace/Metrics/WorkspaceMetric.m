% Base class for different workspace metrics
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
%   All user-defined metrics should implement this base class and
%   define the following:
%       - The minimum and maximum values for the metric
%       - The method to evaluate the metric
%   Any new types of metrics need to be added to the WorkspaceMetricType 
%   enum and also added to the CreateWorkspaceMetric method.
classdef (Abstract) WorkspaceMetric < handle
    
    properties
        type            % Type of joint from JointType enum
    end
    
    properties (SetAccess = protected)
        % Minimum and maximum allowable metric values
        metricMin
        metricMax 
    end
    
    methods
        % Evalute function returns a quantitative evaluation of a metric
        % given dynamics information
        function [metric_cell, comp_time] = evaluate(obj,dynamics,options)
            start_tic = tic;
            f = obj.evaluateFunction(dynamics,options);
            if(f<obj.metricMin)
                metric_cell = {obj.type,obj.metricMin};
            elseif(f>obj.metricMax)
                metric_cell = {obj.type,obj.metricMax};
            else
                metric_cell = {obj.type,f};
            end
            comp_time = toc(start_tic);
        end
        
        % Overrides the metricMin and MetricMax values
        function setMetricLimits(obj,metric_min,metric_max)
            obj.metricMin = metric_min;
            obj.metricMax = metric_max;
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns the metric value
        f = evaluateFunction(obj,dynamics,options);        
    end
    
    methods (Static)
        % Creates a new metric
        function wm = CreateWorkspaceMetric(metricType,desired_set)
            switch metricType
                case WorkspaceMetricType.SEACM
                    wm = SEACM;
                case WorkspaceMetricType.CAPACITY_MARGIN
                    wm = CapacityMarginMetric(desired_set);
                case WorkspaceMetricType.TENSION_FACTOR
                    wm = TensionFactorMetric;
                case WorkspaceMetricType.TENSION_FACTOR_MODIFIED
                    wm = TensionFactorModifiedMetric;
                case WorkspaceMetricType.UNILATERAL_DEXTERITY
                    wm = UnilateralDexterityMetric;
                otherwise
                    error('Workspace metric type is not defined');
            end
            wm.type = metricType;
        end
    end
end