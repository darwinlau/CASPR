% Class to store the configuration of different robots from the XML files
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    This class stores the locations for all of the different CDPRs that
%    are accessible within CASPR. The CDPR information is stored as XML
%    files. New robots that are added must also be added to the 
%    ModelConfigType and models_list.csv to be accessible.
classdef ModelConfig < ModelConfigBase    
    properties (Constant)
        MODEL_FOLDER_PATH = '/models';
        LIST_FILENAME = '/models_list.csv';
    end
    
    methods
        % Constructor for the ModelConfig class. This builds the xml
        % objects.
        function c = ModelConfig(type_string)
            c@ModelConfigBase(type_string, ModelConfig.MODEL_FOLDER_PATH, ModelConfig.LIST_FILENAME);
        end
    end
end

