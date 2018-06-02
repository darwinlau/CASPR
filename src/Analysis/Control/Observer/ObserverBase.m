% Base class for CDPR observers
%
% Author        : Chen SONG
% Created       : 2017
% Description    : 
%   The observer will estimate the system state (q, q_dot) and disturbance
%   (in forms of q_ddot and wrench). Depending on the specific observer,
%   some of the output may be not empty (not available from the observer
%   design). The estimation should also be on the same side as the cable
%   force do in the EoM.
%   The observer is also supposed to be running with a frequency higher or
%   equal to the controller frequency, and lower or equal to the forward
%   dynamics update frequency.
classdef ObserverBase < handle
    properties
        dynModel            % The model of the system
        first_time
        delta_t
    end

    methods
        % A constructor for the controller base.
        function cb = ObserverBase(dyn_model, delta_t)
            cb.dynModel = dyn_model;
            cb.first_time = true;
            cb.delta_t = delta_t;
        end
        % Note: 1. the input argument w_ext_active is the active wrench output
        %       2. the disturbance output will have w_ext_active excluded
        function [q_est, q_dot_est, q_ddot_disturbance_est, wrench_disturbance_est]  = execute(obj, q, q_d, u, w_ext_active, t)
            % note to do necessary model update in the executeFunction
            [q_est, q_dot_est, q_ddot_disturbance_est, wrench_disturbance_est] = obj.executeFunction(q, q_d, u, w_ext_active, t);
        end
    end

    methods (Abstract)
        % An abstract executeFunction for all observers. This should take
        % in the state feedback, latest control input (cable force) and the
        % expected output wrench
        [q_est, q_dot_est, q_ddot_disturbance_est, wrench_disturbance_est] = executeFunction(obj, q, q_d, u, w_ext_active, t);
    end    
end
