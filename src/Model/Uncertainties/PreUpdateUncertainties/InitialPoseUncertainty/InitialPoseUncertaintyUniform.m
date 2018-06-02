% Initial pose uncertainties with uniform probability distribution 
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description    :
%    Initial pose uncertainties with uniform probability distribution.
classdef InitialPoseUncertaintyUniform < InitialPoseUncertaintyBase
    properties
        q_bound_range       % The range of bound on initial pose uncertainty. Format: [q1_lb; q1_ub; ...; qn_lb; qn_ub]
        q_d_bound_range     % The range of derivative bound on initial pose uncertainty. Format: [q1_lb; q1_ub; ...; qn_lb; qn_ub]
    end
    
    methods
        % Constructor
        function ipu = InitialPoseUncertaintyUniform(fk_solver, model, q_initial, q_d_initial, q_bound_range, q_d_bound_range)
            ipu@InitialPoseUncertaintyBase(fk_solver,model,q_initial,q_d_initial);
            ipu.q_bound_range   =   q_bound_range;
            ipu.q_d_bound_range =   q_d_bound_range;
        end
        
        % Apply with the initial offsets
        function [update_q,update_q_dot] = applyInitialOffset(obj,q,q_dot)
            q_length = length(obj.q_bound_range);
%             update_q = q - obj.q_bound_range(1:2:q_length)' + rand(obj.model.numDofs,1).*(obj.q_bound_range(2:2:q_length)'+obj.q_bound_range(1:2:q_length)');
%             update_q_dot = q_dot - obj.q_d_bound_range(1:2:q_length)' + rand(obj.model.numDofs,1).*(obj.q_d_bound_range(2:2:q_length)'+obj.q_d_bound_range(1:2:q_length)');
            update_q = q + obj.q_bound_range(1:2:q_length) + rand(obj.model.numDofs,1).*(obj.q_bound_range(2:2:q_length) - obj.q_bound_range(1:2:q_length));
            update_q_dot = q_dot + obj.q_d_bound_range(1:2:q_length) + rand(obj.model.numDofs,1).*(obj.q_d_bound_range(2:2:q_length) - obj.q_d_bound_range(1:2:q_length));
        end
    end
end