% A function that packs in the FFT algorithm and performs a discrete fast
% fourier transformation.
%
% Auther        : Chen SONG
% Created       : 2018
% Description 	: perform FFT on the input data
%                   data should be arranged in columns

function [fft_series_1, f_1, freq_threshold_1, fft_series_1_summed] = fftAnalysis(t_sig, data_sig, threshold_percentage)

    %% generate the corresponding fourier series
    Fs = length(t_sig)/t_sig(end);
    source_sig_len  =   length(t_sig);
    n               =   2^nextpow2(source_sig_len);
    % do fast fourier transformation to get the fourier transform
    data_ft_sig        =   fft(data_sig, n, 1);
    for i = 1:size(data_sig, 2)
        % derive the two sided spectrum (or fourier series)
        data_fs_2(:,i)        	=   abs(data_ft_sig(:,i)/n);
        % derive the single sided spectrum (or fourier series)
        data_fs_1(:,i)          =   data_fs_2(1:n/2+1,i);
        data_fs_1(2:end-1,i)   	=   2*data_fs_1(2:end-1,i);
    end
    % vector representing the frequency domain of the FFT
    f               =   (Fs*(0:(n/2))/n)';

    fft_series_1    =   data_fs_1;
    f_1             =   f;
    
    %% do result analysis
    % get summed fourier series
    fft_series_1_summed = zeros(size(fft_series_1));
    fft_series_1_summed(1,:) = fft_series_1(1,:);
    for i = 2:size(fft_series_1_summed, 1)
        fft_series_1_summed(i,:) = fft_series_1_summed(i-1,:) + fft_series_1(i,:);
    end
    for i = 1:size(fft_series_1_summed, 2)
        if (fft_series_1_summed(end,i)>0)
            fft_series_1_summed(:,i) = fft_series_1_summed(:,i)/fft_series_1_summed(end,i);
        else
            fft_series_1_summed(:,i) = ones(size(fft_series_1_summed(:,i)));
        end
    end
    % determine a frequency threshold
    % default threshold percentage
    threshold = 0.7;
    if (nargin > 2)
        if threshold_percentage < 0
            threshold_percentage = -threshold_percentage;
        end
        if threshold_percentage > 1
            threshold_percentage = 1;
        end
        threshold = threshold_percentage;
    end
    
    freq_threshold_1            =   zeros(size(fft_series_1_summed,2),1);
    freq_threshold_1_found_flag =   zeros(size(fft_series_1_summed,2),1);
    for i = 1:size(fft_series_1_summed, 1)
        for j = 1:size(fft_series_1_summed, 2)
            if (fft_series_1_summed(i, j) >= threshold)
                if (freq_threshold_1_found_flag(j) == 0)
                    freq_threshold_1_found_flag(j)	=   1;
                    freq_threshold_1(j)         	=   f_1(i);
                end
            end
        end
    end

end