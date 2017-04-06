% Facilities for handling CASPR_configurations in MATLAB.
%
% Author        : Jonathan EDEN
% Created       : 2017
% Description   : A class for loading and saving to CASPR configurations
classdef CASPR_configuration
    methods (Static)
        % Load home path
        function CASPR_homepath_str = LoadHomePath()
            CASPR_homepath = load('CASPR_environment.mat', 'CASPR_homepath');
            CASPR_homepath_str = CASPR_homepath.CASPR_homepath;            
        end
        % Load model config
        function model_config_str = LoadModelConfigPath()
            model_config = load('CASPR_environment.mat', 'CASPR_model_config_path');
            model_config_str = model_config.CASPR_model_config_path;            
        end
    end
end
