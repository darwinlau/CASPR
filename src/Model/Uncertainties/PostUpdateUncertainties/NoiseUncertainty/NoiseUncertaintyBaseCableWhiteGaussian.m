% Noise uncertainties in cable space with white gaussian noise 
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Noise uncertainties to be applied on cable lengths and velocities with white gaussian noise.
classdef NoiseUncertaintyBaseCableWhiteGaussian < NoiseUncertaintyBase
    properties
        nx_specification_vec        % The power of WGN, in dB or variance of WGN. Format: [p_1; p_2; ...; p_m]
        nx_dot_specification_vec    % The power of WGN, in dB or variance of WGN. Format: [p_1; p_2; ...; p_m]
                                    % m is the number of cables
        use_power_of_signal
    end
    
    methods
        % Constructor
        function ipu = NoiseUncertaintyBaseCableWhiteGaussian(model, nx_specification_vec, nx_dot_specification_vec, use_power_of_signal)
            ipu@NoiseUncertaintyBase(model);
            ipu.flag_apply_noise            =   true;
            ipu.use_power_of_signal         =   use_power_of_signal;
            ipu.nx_specification_vec       	=   nx_specification_vec;
            ipu.nx_dot_specification_vec   	=   nx_dot_specification_vec;
            % check the validity of the given disturbance parameters
            % mostly check the dimension
            if (ipu.model.numCables ~= length(nx_specification_vec))
                disp('Dimension of the position noise signal specification vector does not match the number of cables in the robot, no noise will be applied.');
                ipu.flag_apply_noise            =   false;
                ipu.nx_specification_vec        =   [];
                ipu.nx_dot_specification_vec	=   [];
            end
            if (ipu.model.numCables ~= length(nx_dot_specification_vec))
                disp('Dimension of the velocity noise signal specification vector does not match the number of cables in the robot, no noise will be applied.');
                ipu.flag_apply_noise    =   false;
                ipu.nx_specification_vec        =   [];
                ipu.nx_dot_specification_vec	=   [];
            end
        end
        
        % Apply with the cable feedback noise
        function [l_noise, l_dot_noise] = applyFeedbackNoise(obj, ~)
            % the default disturbance is zero
            l_noise     = zeros(obj.model.numCables, 1);
            l_dot_noise = zeros(obj.model.numCables, 1);
            % change if the noise definition is valid
            if (obj.flag_apply_noise)
                for i = 1:obj.model.numCables
                    if (obj.use_power_of_signal)
                        l_noise(i)      =   wgn(1, 1, obj.nx_specification_vec(i));
                        l_dot_noise(i)  =   wgn(1, 1, obj.nx_dot_specification_vec(i));
                    else
                        l_noise(i)      =   sqrt(obj.nx_specification_vec(i))*randn(1,1);
                        l_dot_noise(i)  =   sqrt(obj.nx_dot_specification_vec(i))*randn(1,1);
                    end
                end
            end
        end
    end
end