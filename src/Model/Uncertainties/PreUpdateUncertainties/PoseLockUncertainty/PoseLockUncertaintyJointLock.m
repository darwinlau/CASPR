% Pose lock uncertainties within joint space. 
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Pose lock uncertainties within joint space.
classdef PoseLockUncertaintyJointLock < PoseLockUncertaintyBase
    properties
        t_lock_sequence     % Describes when to lock the pose, and when to release. Format: [t_lock_1; t_unlock_1; ...; t_lock_m; t_unlock_m]
        flag_use_lock       % If t_lock_sequence is defined properly, this flag will be enabled, otherwise not
        pose_lock_effective     % Flag to show whether or not we're going to apply pose lock
    end
    
    methods
        % Constructor
        function ipu = PoseLockUncertaintyJointLock(model, t_sequence)
            ipu@PoseLockUncertaintyBase(model);
            ipu.pose_lock_effective = false;
            ipu.t_lock_sequence =   t_sequence;
            ipu.flag_use_lock   =   true;
            if (mod(length(t_sequence), 2))
                ipu.t_lock_sequence = ipu.t_lock_sequence(1:length(t_sequence) - 1);
            end
            % check the validity of the lock time sequence, if invalid then
            % apply no pose lock
            if (length(t_sequence) <= 1)
                disp('Invalid lock time sequence, no pose lock will be applied');
                ipu.flag_use_lock   =   false;
            end
            if (sum(ipu.t_lock_sequence(1:2:length(ipu.t_lock_sequence) - 1) > ipu.t_lock_sequence(2:2:length(ipu.t_lock_sequence))) > 0)
                disp('Invalid lock time sequence, no pose lock will be applied');
                ipu.flag_use_lock   =   false;
            end
        end
        
        % Apply with the initial offsets
        function [is_locked] = applyPoseLock(obj,t)
            obj.pose_lock_effective = false;
            seq_len = floor(length(obj.t_lock_sequence)/2);
            if (obj.flag_use_lock)
                for i = 1:seq_len
                    if (t >= obj.t_lock_sequence(2*i - 1)) && (t < obj.t_lock_sequence(2*i))
                        obj.pose_lock_effective = true;
                    end
                end
            else
                obj.pose_lock_effective = false;
                disp('Invalid lock time sequence, no lock will be applied')
            end
            is_locked = obj.pose_lock_effective;
        end
    end
end