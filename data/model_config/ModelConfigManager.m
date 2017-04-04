% Manager class for the library of configs
%
% Author        : Darwin LAU
% Created       : 2017
% Description   :
%    Serves to manage the ModelConfig and DevModelConfig
classdef ModelConfigManager
    methods (Static)
        % Get the list of model names from the ModelConfig list's CSV
        function models = GetModelConfigListNames()        
            root_folder = [fileparts(mfilename('fullpath')), ModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, ModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        % Get the list of development model names from the DevModelConfig list's CSV
        function models = GetDevModelConfigListNames()        
            root_folder = [fileparts(mfilename('fullpath')), DevModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, DevModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        % Get the list of test model names from the TestModelConfig list's CSV
        function models = GetTestModelConfigListNames()        
            root_folder = [fileparts(mfilename('fullpath')), TestModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, TestModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        function AddModelConfig(name, folder, bodies_xml_filename, cable_xml_filename, trajectory_xml_filename)
            % Check if name already exists in the model config list
            
            % Add line to the CSV file
            
            % Create file structure: add folder, add bodies, cable and
            % trajectory XML files from template (copy from template)
        end
        
        function RemoveModelConfig()
            % Check if name is in the model config list
            
            % Remove line from the CSV file
            
            % Clean up file structure, delete folder and files inside
        end
        
        function AddDevModelConfig(name, folder, bodies_xml_filename, cable_xml_filename, trajectory_xml_filename)
            % Check if name already exists in the model config list
            
            % Add line to the CSV file
            
            % Create file structure: add folder, add bodies, cable and
            % trajectory XML files from template (copy from template)
        end
        
        function RemoveDevModelConfig()
            % Check if name is in the model config list
            
            % Remove line from the CSV file
            
            % Clean up file structure, delete folder and files inside
        end
    end
end

