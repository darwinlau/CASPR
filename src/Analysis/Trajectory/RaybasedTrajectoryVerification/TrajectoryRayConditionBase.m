% Base class for different ray workspace conditions
%
% Author        : Zeqing ZHANG
% Created       : 2019
% Description:

classdef TrajectoryRayConditionBase < handle
    properties 
        method          % Method of implementation (an enum)
        type            % Type of joint from JointType enum
    end
    
    methods
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        function [condition_type, condition_intervals, comp_time] = evaluate(obj,dynamics,traj)
            start_tic       = tic;
            condition_intervals = obj.evaluateFunction(dynamics,traj);
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
        function wc = CreateTrajectoryRayCondition(conditionType,min_ray_percentage,q_dof_type)
            switch conditionType
                case TrajectoryRayConditionType.WRENCH_CLOSURE
                    wc = TrajWrenchClosureRayCondition(min_ray_percentage,q_dof_type);
                case TrajectoryRayConditionType.CABLE_INTERFERENCE_FREE
                    wc = TrajCableInterferenceFreeRayCondition(min_ray_percentage,q_dof_type);
                otherwise
                    CASPR_log.Error('Workspace condition type is not defined');
            end
            wc.type = conditionType;
        end
    end
end