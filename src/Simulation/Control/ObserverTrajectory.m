% Represents a trajectory of disturbances (in joint space)
%
% Author        : Chen SONG
% Created       : 2017
% Description   : 
%   This is a trajectory of joint space disturbances, will probably mainly
%   be used in simulators with disturbance observer
%   Not sure if this should even exist
classdef ObserverTrajectory < TrajectoryBase
    properties
        q_est               % Estimated joint pose (if available)
        q_dot_est           % Estimated joint velocity (if available)
        q                   % Actual (ideal or from FK) joint pose
        q_dot               % Actual (ideal or from FK) joint velocity
        % the external wrench and the external acceleration should be
        % equivalent: w = M(q)*q_dd
        w_ext               % The actual external wrench (if available)
        w_ext_est           % The estimated external wrench
        q_ddot_ext          % The actual disturbance acceleration (if available)
        q_ddot_ext_est      % The equivalent disturbance acceleration
    end
end