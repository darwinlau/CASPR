% Base class for CDPR observers
%
% Author        : Chen SONG
% Created       : 2017
% Description    : 
%   The observer is supposed to estimate the equivalent acceleration for
%   the disturbance wrench. The estimation should also be on the same side
%   as cable force do in the EoM
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

        function [disturbance]  = execute(obj, q, q_d, u, t)
            obj.dynModel.update(q, q_d, q_dd, zeros(obj.dynModel.numDofs,1));
            [disturbance] = obj.executeFunction(q, q_d, u, t);
        end
    end

    methods (Abstract)
        % An abstract executeFunction for all controllers. This should take
        % in the generalised coordinate information and produces a control
        % input.
        [disturbance] = executeFunction(obj, q, q_d, u, t);
    end    
end
