% Constant initial pose uncertainties 
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Constant initial pose uncertainties.
classdef InitialPoseUncertaintyConstant < InitialPoseUncertaintyBase
    properties
        q_err       % The joint pose initial error
        q_d_err     % The joint velocity initial error
    end
    
    methods
        % Constructor
        function ipu = InitialPoseUncertaintyConstant(fk_solver, model, q_initial, q_d_initial, q_err, q_d_err)
            ipu@InitialPoseUncertaintyBase(fk_solver,model,q_initial,q_d_initial);
            ipu.q_err   =   q_err;
            ipu.q_d_err =   q_d_err;
        end
        
        % Apply with the initial offsets
        function [update_q,update_q_dot] = applyInitialOffset(obj,q,q_dot)
            update_q = q + obj.q_err;
            update_q_dot = q_dot + obj.q_d_err;
        end
    end
end