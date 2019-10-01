% Enum for the type of trajectory
%
% Author        : Paul Cheng
% Created       : 2019
% Description   : 3 types of trajectories are supported. For other
% trajectory, consider to use pose wise input

classdef TrajectoryType < handle
    enumeration 
        PARAMETRIC
        POSE_WISE
        TRIGONOMETRIC
        CONTROL_POINTS
    end
end