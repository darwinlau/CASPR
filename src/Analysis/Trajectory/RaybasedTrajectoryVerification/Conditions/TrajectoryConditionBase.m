% Base class for different trajectories conditions
%
% Author        : Paul Cheng
% Created       : 2019
% Description   :
classdef TrajectoryConditionBase < handle
    properties (Constant, Abstract)
        % Type of workspace condition (WorkspaceConditionType enum)
        type
    end
    
    properties 
%         sample_number          % number of sample point(affect accuracy)
    end
    
    methods
        % The unified implemetnation of evaluate. This evaluates the object
        % dynamics to determine if the workspace condition is satisfied.
        function [traj_interval,time_interval, comp_time,other_info,theta] = evaluate(obj,model,trajectory,time_range,maximum_trajectory_degree)
            start_tic       = tic;
            [traj_interval,time_interval,other_info,theta] = obj.evaluateFunction(model,trajectory,time_range,maximum_trajectory_degree);
            comp_time = toc(start_tic);
        end
    end
    
    methods (Abstract)
        % evalute - This function takes in the workspace dynamics and
        % returns a boolean for whether the point lies in the workspace.
        f = evaluateFunction(obj, evaluated_trajectory);
    end
    
    methods (Static)
        % Creates a new condition
        function tc = CreateTrajectoryCondition(method,conditionType, sampe_sizes,min_cable_dist)
            switch conditionType
                case TrajectoryConditionType.WRENCH_CLOSURE
                    tc = TrajecotryWrenchClousreCondition(method,sampe_sizes);
                case TrajectoryConditionType.CABLE_INTERFERENCE_FREE
                    tc = TrajectoryCableInterferenceFreeCondition(method,sampe_sizes,min_cable_dist);
                otherwise
                    CASPR_log.Print('Trajectory condition type is not defined',CASPRLogLevel.ERROR);
            end
        end
         
    end
end