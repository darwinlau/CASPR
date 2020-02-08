% Base class for different workspace conditions
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description:
%   All user-defined conditions should implement this base class and
%   define the following:
%       - The method to evaluate the workspace condition
%   Any new types of conditions need to be added to the WorkspaceConditionType 
%   enum and also added to the CreateWorkspaceCondition method.
classdef WorkspaceConditionBase < handle
    properties (Constant, Abstract)
        % Type of workspace condition (WorkspaceConditionType enum)
        type
    end
    
    properties 
        method          % Method of implementation (an enum)
    end
    
    methods
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        function [condition_value, comp_time] = evaluate(obj, dynamics, evaluated_metrics)
            start_tic       = tic;
            condition_value = obj.evaluateFunction(dynamics, evaluated_metrics);
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, dynamics, evaluated_metrics);
    end
    
    methods (Static)
        % Creates a new condition
        function wc = CreateWorkspaceCondition(conditionType, method)
            switch conditionType
                case WorkspaceConditionType.WRENCH_CLOSURE
                    wc = WrenchClosureCondition(method);
                case WorkspaceConditionType.WRENCH_FEASIBLE
                    CASPR_log.Error('Wrench-Feasible Workspace (WFW) cannot be created through CreateWorkspaceCondition and must be instantiated directly through WrenchFeasibleCondition(desired_set, method)');
                case WorkspaceConditionType.STATIC
                    wc = WorkspaceStaticCondition(method);
                case WorkspaceConditionType.INTERFERENCE
                    wc = InterferenceFreeCondition(method);
                otherwise
                    CASPR_log.Print('Workspace condition type is not defined', CASPRLogLevel.ERROR);
            end
        end
    end
end