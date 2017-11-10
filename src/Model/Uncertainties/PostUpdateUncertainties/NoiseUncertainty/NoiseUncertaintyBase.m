% Base class for uncertainties related to feedback noise injection
%
% Author        : Chen SONG
% Created       : 2017
% Description    :
%    Base class for uncertainties related to feedback noise injection
classdef (Abstract) NoiseUncertaintyBase < PostUpdateUncertaintyBase
    properties
        model                       % Model object
        flag_apply_noise            % If the noise definition checked out, noise will be applied
    end
    
    methods
        
        function ipu = NoiseUncertaintyBase(model) 
            ipu.model           =   model;
        end
        
    end
    methods(Abstract)      
        % Inject noise in feedback
        [noise_x, noise_x_dot] = applyFeedbackNoise(obj,t);        
    end
end

