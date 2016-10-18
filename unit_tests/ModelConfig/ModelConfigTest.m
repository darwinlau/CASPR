% Unit test of the model configuration.
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    Unit tests to confirm that the model configuration is correct.
classdef ModelConfigTest < matlab.unittest.TestCase
    methods (Test)
        % Test that the master_list is correctly setup
        function modelListValidTest(testCase)
            disp('Testing masterListValid');
            load('CASPR_environment.mat', 'home_path');
            % Open up and scan through the master list
            fid = fopen([home_path,'/data/model_config/models/models_list.csv']);
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            unique_cell_array = unique(cell_array{1});
            % Two tests are conducted firstly are there any repeated files
            % in the materlist
            assert(length(cell_array{1}) == length(unique_cell_array), 'There are repeated identifiers in the master list');
            % Secondly does each iterm in the list map to an enum.
            for i = 1:length(cell_array{1})
                % Check if the enum exists
                ModelConfigType.(cell_array{1}{i});
            end
        end

        % Test that the model configuration files exist.
        function modelConfigFilesExistTest(testCase)
            disp('Testing that model config files exist');
            [mSet, mNames] = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                disp(['Testing ModelConfigType: ', mNames{i}]);
                m = ModelConfig(mSet(i));
                testCase.assertNotEmpty(m);
            end
        end

        % Test that all models can be constructed as bodies.
        function modelCreation(testCase)
            disp('Testing model creation');
            [mSet, mNames] = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                disp(['Testing ModelConfigType: ', mNames{i}]);
                m = ModelConfig(mSet(i));
                model = m.getModel(m.defaultCableSetId);
                testCase.assertNotEmpty(model);
            end
        end

        % Test that the test master_list is correctly setup
        function testModelListValidTest(testCase)
            disp('testModelListValid test');
            load('CASPR_environment.mat', 'home_path');
            % Open up and scan through the master list
            fid = fopen([home_path,'/data/model_config/test_models/test_models_list.csv']);
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            unique_cell_array = unique(cell_array{1});
            % Two tests are conducted firstly are there any repeated files
            % in the materlist
            assert(length(cell_array{1}) == length(unique_cell_array), 'There are repeated identifiers in the master list');
            % Secondly does each item in the list map to an enum.
            for i = 1:length(cell_array{1})
                % Check if the enum exists
                TestModelConfigType.(cell_array{1}{i});
            end
        end

        % Test that the test model configuration files exist.
        function testModelConfigFilesExistTest(testCase)
            disp('testModelConfigFilesExist test');
            [mSet, mNames] = enumeration('TestModelConfigType');
            for i = 1:length(mSet)
                % disp(['Testing ModelConfigType: ', mNames{i}]);
                m = TestModelConfig(mSet(i));
                testCase.assertNotEmpty(m);
            end
        end

        % Test that all test models can be constructed as bodies.
        function testModelCreationTest(testCase)
            disp('testModelBodyCreation test');
            [mSet, mNames] = enumeration('TestModelConfigType');
            for i = 1:length(mSet)
                % disp(['Testing ModelConfigType: ', mNames{i}]);
                m = TestModelConfig(mSet(i));
                model = m.getModel(m.defaultCableSetId);
                testCase.assertNotEmpty(model);
            end
        end


        function devModelListValidTest(testCase)
            disp('devModelListValid test');
            load('CASPR_environment.mat', 'home_path');
            % Open up and scan through the master list
            fid = fopen([home_path,'/data/model_config/indev_models/indev_models_list.csv']);
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            unique_cell_array = unique(cell_array{1});
            % Two tests are conducted firstly are there any repeated files
            % in the materlist
            assert(length(cell_array{1}) == length(unique_cell_array), 'There are repeated identifiers in the master list');
            % Secondly does each item in the list map to an enum.
            for i = 1:length(cell_array{1})
                % Check if the enum exists
                DevModelConfigType.(cell_array{1}{i});
            end
        end

        % Test that the test model configuration files exist.
        function devModelConfigFilesExistTest(testCase)
            disp('devModelConfigFilesExist test');
            [mSet, mNames] = enumeration('DevModelConfigType');
            for i = 1:length(mSet)
                % disp(['Testing ModelConfigType: ', mNames{i}]);
                m = DevModelConfig(mSet(i));
                testCase.assertNotEmpty(m);
            end
        end

        % Test that all test models can be constructed as bodies.
        function devModelCreationTest(testCase)
            disp('devModelBodyCreation test');
            [mSet, mNames] = enumeration('DevModelConfigType');
            for i = 1:length(mSet)
                % disp(['Testing ModelConfigType: ', mNames{i}]);
                m = DevModelConfig(mSet(i));
                model = m.getModel(m.defaultCableSetId);
                testCase.assertNotEmpty(model);
            end
        end
    end
end
