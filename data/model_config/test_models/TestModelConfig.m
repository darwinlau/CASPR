% The model configuration for the test suite robots
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef TestModelConfig < ModelConfigBase    
    properties (Constant)
        MODEL_FOLDER_PATH = '/test_models';
        LIST_FILENAME = '/test_models_list.csv';
    end
    
    methods
        % Constructor for the ModelConfig class. This builds the xml
        % objects.
        function c = TestModelConfig(type)
            c@ModelConfigBase(type, TestModelConfig.MODEL_FOLDER_PATH, TestModelConfig.LIST_FILENAME);
        end
    end
end

