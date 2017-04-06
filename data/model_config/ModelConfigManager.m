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
            root_folder = [CASPR_configuration.LoadModelConfigPath(), ModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, ModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        % Get the list of development model names from the DevModelConfig list's CSV
        function models = GetDevModelConfigListNames()        
            root_folder = [CASPR_configuration.LoadModelConfigPath(), DevModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, DevModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        % Get the list of test model names from the TestModelConfig list's CSV
        function models = GetTestModelConfigListNames()        
            root_folder = [CASPR_configuration.LoadModelConfigPath(), TestModelConfig.MODEL_FOLDER_PATH];            
            % Open the master list
            fid = fopen([root_folder, TestModelConfig.LIST_FILENAME]);
            
            % Determine the Filenames
            % Load the contents
            csv_content = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            models = csv_content{1};            
        end
        
        function AddModelConfig(name, folder, bodies_xml_filename, cable_xml_filename, trajectory_xml_filename)
            % Determine the base folder and root folder
            base_folder = CASPR_configuration.LoadModelConfigPath();
            root_folder = [base_folder, ModelConfig.MODEL_FOLDER_PATH];
            % Check if name already exists in the model config list
            model_list = ModelConfigManager.GetModelConfigListNames();
            for i = 1:length(model_list)
                CASPR_log.Assert(~strcmp(model_list{i},name),'Model name already exists in CASPR');
            end
            % Create file structure: add folder, add bodies, cable and
            % trajectory XML files from template (copy from template)
            % Confirm that the directory exists
            dir_folder = [root_folder,folder];
            CASPR_log.Assert(~isdir(dir_folder),'Directory already exists');
            % Add the folder
            status = mkdir(dir_folder);
            CASPR_log.Assert(status,'Invalid folder name');
            % Copy template files
            copyfile([base_folder,'/templates/template_bodies.xml'],[dir_folder,bodies_xml_filename])
            copyfile([base_folder,'/templates/template_cables.xml'],[dir_folder,cable_xml_filename])
            copyfile([base_folder,'/templates/template_trajectories.xml'],[dir_folder,trajectory_xml_filename])
            
            % Add line to the CSV file
            fid = fopen([root_folder, ModelConfig.LIST_FILENAME],'a');
            fprintf(fid,[name,',',folder,',',bodies_xml_filename,',',cable_xml_filename,',',trajectory_xml_filename,',','','\n']);
            fclose(fid);
        end
        
        function RemoveModelConfig(name)
            % Check if name is in the model config list
            model_list = ModelConfigManager.GetModelConfigListNames();
            name_exists = 0;
            number_models = length(model_list);
            for i = 1:number_models
                if(strcmp(model_list{i},name))
                    name_exists = 1;
                    remove_index = i;
                end
            end
            CASPR_log.Assert(name_exists,'Model name not found');
            % Remove line from the CSV file
            root_folder = [CASPR_configuration.LoadModelConfigPath(), ModelConfig.MODEL_FOLDER_PATH];
            file_path = [root_folder, ModelConfig.LIST_FILENAME];
            fid = fopen(file_path,'r');
            str_array = cell(number_models-1,1);
            write_index = 1;
            for i=1:number_models
                if(i ~= remove_index)
                    str_array{write_index} = fgetl(fid);
                    write_index = write_index + 1;
                else
                    remove_line = strsplit(fgetl(fid),',');
                end
            end
            fclose(fid);
            fid = fopen(file_path,'w');
            for i=1:number_models-1
                fprintf(fid,'%s\n',str_array{i});
            end
            fclose(fid);
            % Clean up file structure, delete folder and files inside
            dir_folder = [root_folder,remove_line{2}]; % The second entry is the folder string
            CASPR_log.Assert(isdir(dir_folder),'No Directory exists');
            command = ['rm -rf ',dir_folder];
            system(command);
        end
        
        function AddDevModelConfig(name, folder, bodies_xml_filename, cable_xml_filename, trajectory_xml_filename)
            % Determine the base folder and root folder
            base_folder = CASPR_configuration.LoadModelConfigPath();
            root_folder = [base_folder, DevModelConfig.MODEL_FOLDER_PATH];
            % Check if name already exists in the model config list
            model_list = ModelConfigManager.GetDevModelConfigListNames();
            for i = 1:length(model_list)
                CASPR_log.Assert(~strcmp(model_list{i},name),'Model name already exists in CASPR');
            end
            % Create file structure: add folder, add bodies, cable and
            % trajectory XML files from template (copy from template)
            % Confirm that the directory exists
            dir_folder = [root_folder,folder];
            CASPR_log.Assert(~isdir(dir_folder),'Directory already exists');
            % Add the folder
            status = mkdir(dir_folder);
            CASPR_log.Assert(status,'Invalid folder name');
            % Copy template files
            copyfile([base_folder,'/templates/template_bodies.xml'],[dir_folder,bodies_xml_filename])
            copyfile([base_folder,'/templates/template_cables.xml'],[dir_folder,cable_xml_filename])
            copyfile([base_folder,'/templates/template_trajectories.xml'],[dir_folder,trajectory_xml_filename])
            
            % Add line to the CSV file
            fid = fopen([root_folder, DevModelConfig.LIST_FILENAME],'a');
            fprintf(fid,[name,',',folder,',',bodies_xml_filename,',',cable_xml_filename,',',trajectory_xml_filename,',']);
            fclose(fid);            
        end
        
        function RemoveDevModelConfig(name)
            % Check if name is in the model config list
            model_list = ModelConfigManager.GetDevModelConfigListNames();
            name_exists = 0;
            number_models = length(model_list);
            for i = 1:number_models
                if(strcmp(model_list{i},name))
                    name_exists = 1;
                    remove_index = i;
                end
            end
            CASPR_log.Assert(name_exists,'Model name not found');
            % Remove line from the CSV file
            root_folder = [CASPR_configuration.LoadModelConfigPath(), DevModelConfig.MODEL_FOLDER_PATH];
            file_path = [root_folder, DevModelConfig.LIST_FILENAME];
            fid = fopen(file_path,'r');
            str_array = cell(number_models-1,1);
            write_index = 1;
            for i=1:number_models
                if(i ~= remove_index)
                    str_array{write_index} = fgetl(fid);
                    write_index = write_index + 1;
                else
                    remove_line = strsplit(fgetl(fid),',');
                end
            end
            fclose(fid);
            fid = fopen(file_path,'w');
            for i=1:number_models-1
                fprintf(fid,'%s\n',str_array{i});
            end
            fclose(fid);
            % Clean up file structure, delete folder and files inside
            dir_folder = [root_folder,remove_line{2}]; % The second entry is the folder string
            CASPR_log.Assert(isdir(dir_folder),'No Directory exists');
            command = ['rm -rf ',dir_folder];
            system(command);
        end
    end
end

