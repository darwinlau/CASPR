% Solves the forward kinematics using the integration approach by taking
% the pseudo-inverse solution of the derivative kinematics relationship.
%
% Author        : Darwin LAU
% Created       : 2015
% Description    : 
classdef FKDifferential < FKAnalysisBase
    
    properties (Access = private)
    end
    
    methods
        % The constructor for this solver.  This is simply a call of the
        % base constructor.
        function fkd = FKDifferential(model)
            fkd@FKAnalysisBase(model);
        end
        
        % The implementation of the abstract computeFunction methods.
        function [q, q_dot] = computeFunction(obj, length, lengths_prev_2, q_prev, ~, delta_t)
            if delta_t ~= 0
                L_pinv = (obj.model.L' * obj.model.L) \ obj.model.L';
                q_dot = L_pinv * (length - lengths_prev_2)/(2*delta_t);
                %q_dot = pinv(obj.model.L) * (length - lengths_prev_2)/(2*delta_t);
            else
                q_dot = zeros(size(q_prev));
            end
            q = q_prev + q_dot * delta_t;
        end
    end
end

