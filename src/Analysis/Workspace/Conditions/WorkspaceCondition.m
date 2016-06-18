% Base class for different workspace conditions
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description:
%   All user-defined metrics should implement this base class and
%   define the following:
%       - The method to evaluate the metric
%       - The method to determine if neighbouring points are connected
%   Any new types of metrics need to be added to the WorkspaceConditionType 
%   enum and also added to the CreateWorkspaceCondition method.
classdef WorkspaceCondition < handle
    properties 
        method          % Method of implementation (an enum)
        type            % Type of joint from JointType enum
    end
    
    methods
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        function [condition_type, condition_value, comp_time] = evaluate(obj,dynamics,workspace_point)
            start_tic       = tic;
            condition_value = obj.evaluateFunction(dynamics, workspace_point);
            condition_type  = obj.type;
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, dynamics, workspace_point);
    end
    
    methods (Static)
        % Creates a new condition
        function wc = CreateWorkspaceCondition(conditionType,method,desired_set)
            switch conditionType
                case WorkspaceConditionType.WRENCH_CLOSURE
                    wc = WrenchClosure(method);
                case WorkspaceConditionType.WRENCH_FEASIBLE
                    wc = WrenchFeasible(method,desired_set);
                case WorkspaceConditionType.STATIC
                    wc = WorkspaceStatic(method);
                otherwise
                    error('Workspace metric type is not defined');
            end
            wc.type = conditionType;
        end
    end
end