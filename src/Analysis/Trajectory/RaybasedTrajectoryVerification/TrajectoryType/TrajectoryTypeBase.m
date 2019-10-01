% Base class for different trajectories conditions
%
% Author        : Paul Cheng
% Created       : 2019
% Description:
%   All user-defined conditions should implement this base class and
%   define the following:
%       - The method to evaluate the trajectory condition
%   Any new types of conditions need to be added to the TrajectoryConditionType 
%   enum and also added to the CreateTrajectoryCondition method.
classdef TrajectoryTypeBase < handle
    properties (Constant, Abstract)
        % Type of workspace condition (WorkspaceConditionType enum)
        type
    end

    methods (Static)
        % Creates a new condition
        function type = CreateTrajectoryType(trajectoryType)
            switch trajectoryType
                case TrajectoryType.PARAMETRIC
                    type = TrajectoryType.PARAMETRIC;
                case TrajectoryType.POSE_WISE
                    type = TrajectoryType.POSE_WISE;
                case TrajectoryType.TRIGONOMETRIC
                    type = TrajectoryType.POSE_WISE;
                otherwise
                    CASPR_log.Print('Trajectory type type is not valid',CASPRLogLevel.ERROR);
            end
        end
         
    end
end