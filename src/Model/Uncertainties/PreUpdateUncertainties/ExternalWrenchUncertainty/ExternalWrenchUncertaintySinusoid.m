% External wrench disturbance uncertainties with sinusoid disturbance
% functions
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    External wrench disturbance uncertainties with discrete sequence with
%    sinusoid disturbance functions
%    The disturbance is considered as being applied to the right hand side
%    of the EoM.
classdef ExternalWrenchUncertaintySinusoid < ExternalWrenchUncertaintyBase
    properties
        amplitude_vec               % The vector contains amplitudes of sinusoid disturbances in different channels. Format: [a_1; a_2; ...; a_m]
        phase_vec                   % The vector contains initial phases of sinusoid disturbances in different channels. Format: [phi_1; phi_2; ...; phi_m]
        angular_velocity_vec        % The vector contains angular velocities of sinusoid disturbances in different channels. Format: [w_1; w_2; ...; w_m]
    end
    
    methods
        % Constructor
        function ipu = ExternalWrenchUncertaintySinusoid(model, amplitude_vec, phase_vec, angular_velocity_vec)
            ipu@ExternalWrenchUncertaintyBase(model);
            ipu.amplitude_vec           =   amplitude_vec;
            ipu.phase_vec               =   phase_vec;
            ipu.angular_velocity_vec    =   angular_velocity_vec;
            ipu.flag_apply_disturbance  =   true;
            % check the validity of the given disturbance parameters
            % mostly check the dimension
            if (ipu.model.numDofs ~= length(amplitude_vec))
                disp('Dimension of the given disturbance amplitude vector does not match that of the robot, no disturbance will be applied.');
                ipu.flag_apply_disturbance = false;
                ipu.amplitude_vec           =   [];
                ipu.phase_vec               =   [];
                ipu.angular_velocity_vec    =   [];
            end
            if (ipu.model.numDofs ~= length(phase_vec))
                disp('Dimension of the given disturbance phase vector does not match that of the robot, no disturbance will be applied.');
                ipu.flag_apply_disturbance = false;
                ipu.amplitude_vec           =   [];
                ipu.phase_vec               =   [];
                ipu.angular_velocity_vec    =   [];
            end
            if (ipu.model.numDofs ~= length(angular_velocity_vec))
                disp('Dimension of the given disturbance angular veclocity vector does not match that of the robot, no disturbance will be applied.');
                ipu.flag_apply_disturbance = false;
                ipu.amplitude_vec           =   [];
                ipu.phase_vec               =   [];
                ipu.angular_velocity_vec    =   [];
            end
        end
        
        % Apply with the sinusoidal disturbance
        function [w_ext] = applyWrechDisturbance(obj, t)
            % the default disturbance is zero
            w_ext = zeros(obj.model.numDofs, 1);
            % change the disturbance if input sequences are valid
            if (obj.flag_apply_disturbance)
                for i = 1:obj.model.numDofs
                    w_ext(i) = -obj.amplitude_vec(i)*sin(obj.angular_velocity_vec(i)*t + obj.phase_vec(i));
                end
            end
        end
    end
end