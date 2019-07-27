% Class for avoiding joint limits 
%
% Author        : Dominic Chan
% Created       : 2019
% Description   : This formulation focus on generality, not efficiency
classdef AvoidJointLimits < AvoidanceBase
    properties        
        buffer              % Vector - buffer before hitting joint limit
        q_max_limit         % Vector - max q 
        q_min_limit         % Vector - min q
        q_max_buffer        % Vector - max q with buffer
        q_min_buffer        % Vector - min q with buffer
    end

    methods
        % Constructor
        function ajl = AvoidJointLimits(cdpr, dt, k_q, epsilon, buffer)            
            ajl@AvoidanceBase(cdpr, dt, k_q, epsilon);           
            CASPR_log.Assert(length(buffer)==cdpr.numDofs, 'Buffer should have the same size with joint space');
            ajl.buffer = buffer; 
            ajl.q_max_limit  = cdpr.bodyModel.q_max;
            ajl.q_min_limit  = cdpr.bodyModel.q_min; 
            ajl.q_max_buffer = cdpr.bodyModel.q_max - buffer;
            ajl.q_min_buffer = cdpr.bodyModel.q_min + buffer; 
        end     
                
        % Evaluate function
        function value = evaluate(obj, q, ~) 
            % Worst-case
            worst_max = min(obj.q_max_buffer - q);
            worst_min = min(q - obj.q_min_buffer);
            value = min(worst_max, worst_min);
        end
        
        % Triggers
        function flag = isAvoidanceObjective(obj, q, q_d)
            q_next = q + q_d*obj.dt;
            if any(q_next < obj.q_min_buffer) || any(q_next > obj.q_max_buffer)
                flag = true;
            else
                flag = false;
            end
        end
        function flag = isAvoidanceConstraints(obj, q, q_d)
            q_next = q + q_d*obj.dt;
            if any(q_next < obj.q_min_limit) || any(q_next > obj.q_max_limit)
                flag = true;
            else
                flag = false;
            end
        end
        
        % Not needed for this function
        function initVariables(~)
        end
    end   
end
