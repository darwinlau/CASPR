% Base class for different ray workspace conditions
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description:
%   All user-defined conditions should implement this base class and
%   define the following:
%       - The method to evaluate the workspace condition
%   Any new types of conditions need to be added to the WorkspaceRayConditionType 
%   enum and also added to the CreateWorkspaceRayCondition method.
classdef WorkspaceRayConditionBase < handle
    properties 
        method          % Method of implementation (an enum)
        type            % Type of joint from JointType enum
    end
    
    methods
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        function [condition_type, condition_intervals, comp_time] = evaluate(obj,dynamics,workspace_ray)
            start_tic       = tic;
            condition_intervals = obj.evaluateFunction(dynamics, workspace_ray);
            condition_type  = obj.type;
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, dynamics, workspace_ray);
    end
    
    methods (Static)
        % Creates a new condition (for the moment methods and wrench sets
        % are not considered)
        function wc = CreateWorkspaceRayCondition(conditionType,min_ray_percentage,q_dof_type)
            switch conditionType
                case WorkspaceRayConditionType.WRENCH_CLOSURE
                    wc = WrenchClosureRay(min_ray_percentage,q_dof_type);
                case WorkspaceRayConditionType.INTERFERENCE
                    wc = InterferenceFreeRay(min_ray_percentage,q_dof_type);
                case WorkspaceRayConditionType.INTERFERENCE_C_E
                    wc = InterferenceFreeRay_C_E(min_ray_percentage,q_dof_type);
                case WorkspaceRayConditionType.INTERFERENCE_C_O
                    wc = InterferenceFreeRay_C_O(min_ray_percentage,q_dof_type);
                otherwise
                    CASPR_log.Error('Workspace condition type is not defined');
            end
            wc.type = conditionType;
        end
    end
end