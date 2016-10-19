% The model configuration for robots that are in development
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef DevModelConfig < ModelConfigBase    
    properties (Constant)
        MODEL_FOLDER_PATH = '/indev_models';
        LIST_FILENAME = '/indev_models_list.csv';
    end
    
    methods
        function c = DevModelConfig(type)
            CASPR_log.Assert(isa(type, 'DevModelConfigType'), 'The ''type'' for ModelConfig must be of the enum ''DevModelConfigType''');
            c@ModelConfigBase(type, DevModelConfig.MODEL_FOLDER_PATH, DevModelConfig.LIST_FILENAME);
        end
    end
end

