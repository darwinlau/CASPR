classdef FKDifferential < FKFunction
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
    end
    
    methods
        function fkd = FKDifferential()
        end
        
        function [q, q_dot] = compute(~, length, lengths_prev_2, q_prev, ~, delta_t, kin_model)
            if delta_t ~= 0
                q_dot = pinv(kin_model.L) * (length - lengths_prev_2)/(2*delta_t);
            else
                q_dot = zeros(size(q_prev));
            end
            q = q_prev + q_dot * delta_t;
        end
    end
        
    methods (Static)
    end
end

