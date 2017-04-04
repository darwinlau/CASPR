% Manager class for the library of configs
%
% Author        : Darwin LAU
% Created       : 2017
% Description   :
%    Serves to manage the ModelConfig and DevModelConfig
classdef ModelConfigManager
    methods (Static)
        function models = GetModelConfigListNames()        
            root_folder = [fileparts(mfilename('fullpath')), ModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, ModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        function models = GetDevModelConfigListNames()        
            root_folder = [fileparts(mfilename('fullpath')), DevModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, DevModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        
        function models = GetTestModelConfigListNames()        
            root_folder = [fileparts(mfilename('fullpath')), TestModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, TestModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
    end
end

