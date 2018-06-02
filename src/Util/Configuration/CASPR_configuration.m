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
        % Load GUI_dev_model_config
        function GUI_dev_model_config = LoadDevModelConfig()
            model_config = load('CASPR_environment.mat', 'CASPR_GUI_dev_model_config');
            GUI_dev_model_config = model_config.CASPR_GUI_dev_model_config;            
        end
        % Load global_ModelMode
        function global_model_mode = LoadGlobalModelMode()
            model_config = load('CASPR_environment.mat', 'global_model_mode');
            global_model_mode = model_config.global_model_mode;
            if ~isa(global_model_mode, 'ModelModeType')
                CASPR_configuration.SetGlobalModelMode(ModelModeType.DEFAULT, false);
                global_model_mode = ModelModeType.DEFAULT;
            end
        end
        % Load reuse_compiled flag
        function reuse_compiled = LoadReuseCompiled()
            model_config = load('CASPR_environment.mat', 'reuse_compiled');
            reuse_compiled = model_config.reuse_compiled;
        end
        % Set the compiled_mode flag
        function SetGlobalModelMode(value, msg) 
            % Check input value
            if ~isa(value,'ModelModeType')
                CASPR_log.Warn('Invalid ModelModeType!');
            end            
            load('CASPR_environment.mat');
            config_details = whos(matfile('CASPR_environment.mat'));            
            global_model_mode = value;
            home_path = CASPR_configuration.LoadHomePath();
            save([home_path,'/data/config/CASPR_environment.mat'], config_details.name);
            if nargin <= 1 || msg
                CASPR_log.Info(['Compiled_mode flag set to ', char(global_model_mode)]);
            end
        end
        % Set the reuse_compiled flag
        function SetReuseCompiled(value)            
            load('CASPR_environment.mat');
            config_details = whos(matfile('CASPR_environment.mat'));            
            reuse_compiled = value;
            home_path = CASPR_configuration.LoadHomePath();
            save([home_path,'/data/config/CASPR_environment.mat'], config_details.name);
            CASPR_log.Info(['Reuse_compiled flag set to ',num2str(value)]);
        end
        % Set the GUI_dev_model_config
        function SetDevModelConfig(value)
            load('CASPR_environment.mat');
            config_details = whos(matfile('CASPR_environment.mat'));            
            CASPR_GUI_dev_model_config = value;
            home_path = CASPR_configuration.LoadHomePath();
            save([home_path,'/data/config/CASPR_environment.mat'], config_details.name);
            CASPR_log.Info(['Reuse_compiled flag set to ',num2str(value)]);
        end
        
    end
end
