% External wrench disturbance uncertainties with discrete sequence 
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    External wrench disturbance uncertainties with discrete sequence
%    The disturbance is considered as being applied to the right hand side
%    of the EoM.
classdef ExternalWrenchUncertaintyDisccreteSequence < ExternalWrenchUncertaintyBase
    properties
        time_sequence               % Time points when disturbance change happens. Format: [t_1; t_2; ...; t_m]
        disturbance_sequence        % Disturbance sequence, assume zero disturbance at the beginning. Format: [w_ext_1'; w_ext_2'; ...; w_ext_m']
        sequence_length             % The length of the disturbance sequence
    end
    
    methods
        % Function that extracts the frequency domain information via FFT.
        % Should be called after the generation of the disturbance sequence.
        function output_data = frequencyDomainAnalysis(obj, sampling_freq)
            % default sampling frequency
            default_sampling_frequency = 1000;
            if (nargin < 2)
                disp('Default sampling frequency will be applied.');
                sampling_frequency = default_sampling_frequency;
            else
                sampling_frequency = floor(abs(sampling_freq));
            end
            
            % generate the source signal for different modes
            disturbance_seq_len = obj.time_sequence(end)*sampling_frequency;
            disturbance_seq_dim = size(obj.disturbance_sequence, 2);
            % sampling time sequence
            t_sampling = 0:1/sampling_frequency:obj.time_sequence(end);
            % Fs being the sampling frequency
            Fs      =   sampling_frequency;
            % extract the sampled sequence
            disturbance_source_sig 	=   zeros(disturbance_seq_len, disturbance_seq_dim);
            for i = 1:disturbance_seq_len
                t = t_sampling(i);
                disturbance_source_sig(i, :) = obj.applyWrechDisturbance(t)';
            end
            
            output_data.t_sampled_data              =   t_sampling;
            output_data.disturbance_sampled_data    =   disturbance_source_sig;
            [output_data.disturbance_fs_1, output_data.f] = fftAnalysis(output_data.t_sampled_data, output_data.q_sampled_data);
            
        end
        % Constructor
        function ipu = ExternalWrenchUncertaintyDisccreteSequence(model, time_sequence, disturbance_sequence)
            ipu@ExternalWrenchUncertaintyBase(model);
            ipu.time_sequence           =   time_sequence;
            ipu.disturbance_sequence    =   disturbance_sequence;
            ipu.flag_apply_disturbance  =   true;
            % check the validity of the given disturbance sequence
            % check the dimension of the disturbance wrench
            if (ipu.model.numDofs ~= size(disturbance_sequence, 2))
                disp('Dimension of the given disturbance does not match that of the robot, no disturbance will be applied.');
                ipu.flag_apply_disturbance = false;
                ipu.time_sequence           =   [];
                ipu.disturbance_sequence    =   [];
                ipu.sequence_length         =   0;
            % check the dimension of the time sequence and the disturbance
            % sequence
            elseif (size(time_sequence, 1) ~= size(disturbance_sequence, 1))
                if (min(size(time_sequence, 1), size(disturbance_sequence, 1)) < 1)
                    disp('The input sequences are not complete, no disturbance will be applied.');
                    ipu.flag_apply_disturbance = false;
                    ipu.time_sequence           =   [];
                    ipu.disturbance_sequence    =   [];
                    ipu.sequence_length         =   0;
                else
                    ipu.time_sequence           =   time_sequence(1:min(size(time_sequence, 1), size(disturbance_sequence, 1)));
                    ipu.disturbance_sequence    =   disturbance_sequence(1:min(size(time_sequence, 1), size(disturbance_sequence, 1)), :);
                    ipu.sequence_length         =   min(size(time_sequence, 1), size(disturbance_sequence, 1));
                end
            else
                if (size(time_sequence, 1) < 1)
                    disp('The input sequences are empty, no disturbance will be applied.');
                    ipu.flag_apply_disturbance = false;
                    ipu.time_sequence           =   [];
                    ipu.disturbance_sequence    =   [];
                    ipu.sequence_length         =   0;
                else
                    ipu.sequence_length         =   size(time_sequence, 1);
                end
            end
        end
        
        % Apply with the discrete sequence disturbance
        function [w_ext] = applyWrechDisturbance(obj, t)
            % the default disturbance is zero
            w_ext = zeros(obj.model.numDofs, 1);
            % change the disturbance if input sequences are valid
            if (obj.flag_apply_disturbance)
                section_flag = -1;
                for i = 1:obj.sequence_length
                    if (t < obj.time_sequence(i))
                        section_flag = i - 1;
                        break;
                    end
                end

                if (section_flag == -1)
                    % in this case we should use the last disturbance in disturbance_sequence
                    section_flag = obj.sequence_length;
                    w_ext = -obj.disturbance_sequence(section_flag, :)';
                elseif (section_flag == 0)
                    % in this case we should use zero disturbance, hence
                    % basically do nothing
                else
                    % in this case we should use one of the disturbances in the
                    % sequence
                    w_ext = -obj.disturbance_sequence(section_flag, :)';
                end
            end
        end
    end
end