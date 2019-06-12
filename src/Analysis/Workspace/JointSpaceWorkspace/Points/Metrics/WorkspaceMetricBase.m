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
classdef (Abstract) WorkspaceMetricBase < handle
    properties (Constant, Abstract)
        % Type of workspace (WorkspaceMetricType enum)
        type
        % Minimum and maximum allowable metric values
        metricMin
        metricMax
    end
    
    methods
        % Evalute function returns a quantitative evaluation of a metric
        % given dynamics information
        function [metric_value, comp_time] = evaluate(obj, dynamics)
            start_tic   =   tic;
            f = obj.evaluateFunction(dynamics);
            if(f<obj.metricMin)
                metric_value = obj.metricMin;
            elseif(f>obj.metricMax)
                metric_value = obj.metricMax;
            else
                metric_value = f;
            end
            comp_time = toc(start_tic);
        end
        
        % Overrides the metricMin and MetricMax values
        function setMetricLimits(obj,metric_min,metric_max)
            CASPR_log.Assert(metric_max >= metric_min,'Maximum must be greater than minimum');
            obj.metricMin = metric_min;
            obj.metricMax = metric_max;
        end
    end
    
    % Comment this away for now, see if it is needed. If not, remove this
    % in the future
%     methods (Static)
%         % Creates a new metric
%         function wm = CreateWorkspaceMetric(metricType,desired_set)
%             switch metricType
%                 case WorkspaceMetricType.SEACM
%                     wm = SEACM;
%                 case WorkspaceMetricType.CAPACITY_MARGIN
%                     wm = CapacityMarginMetric(desired_set);
%                 case WorkspaceMetricType.CONDITION_NUMBER
%                     wm = ConditionNumberMetric;
%                 case WorkspaceMetricType.TENSION_FACTOR
%                     wm = TensionFactorMetric;
%                 case WorkspaceMetricType.TENSION_FACTOR_MODIFIED
%                     wm = TensionFactorModifiedMetric;
%                 case WorkspaceMetricType.UNILATERAL_DEXTERITY
%                     wm = UnilateralDexterityMetric;
%                 case WorkspaceMetricType.MIN_CABLE_CABLE_DISTANCE
%                     wm = MinCableCableDistanceMetric;
%                 case WorkspaceMetricType.UNILATERAL_MAXIMUM_FORCE_AMPLIFICATION
%                     wm = UnilateralMaximumForceAmplificationMetric;
%                 otherwise
%                     CASPR_log.Print('Workspace metric type is not defined',CASPRLogLevel.ERROR);
%             end
%         end
%     end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns the metric value
        f = evaluateFunction(obj, dynamics);        
    end
end