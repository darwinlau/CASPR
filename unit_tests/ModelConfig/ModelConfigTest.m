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
            CASPR_log.Debug('Running ModelConfigTest/modelListValidTest');
            % Open up and scan through the master list
            fid = fopen([CASPR_configuration.LoadHomePath(),'/data/model_config/models/models_list.csv']);
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            unique_cell_array = unique(cell_array{1});
            % Test that there are no repeated identifiers in the master
            % list
            assert(length(cell_array{1}) == length(unique_cell_array), 'There are repeated identifiers in the master list');
            CASPR_log.Debug('Done ModelConfigTest/modelListValidTest');
        end

        % Test that the model configuration files exist.
        function modelConfigFilesExistTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/modelConfigFilesExistTest');
            [mSet] = ModelConfigManager.GetModelConfigListNames();
            for i = 1:length(mSet)
                CASPR_log.Debug(['Testing model: ', mSet{i}]);
                m = ModelConfig(mSet{i});
                testCase.assertNotEmpty(m);
            end
            CASPR_log.Debug('Done ModelConfigTest/modelConfigFilesExistTest');
        end

        % Test that all models can be constructed as bodies.
        function modelCreation(testCase)
            CASPR_log.Debug('Running ModelConfigTest/modelCreation');
            [mSet] = ModelConfigManager.GetModelConfigListNames();
            for i = 1:length(mSet)
                CASPR_log.Debug(['Testing model: ', mSet{i}]);
                m = ModelConfig(mSet{i});
                model = m.getModel(m.defaultCableSetId);
                testCase.assertNotEmpty(model);
            end
            CASPR_log.Debug('Done ModelConfigTest/modelCreation');
        end
        
        % Test that all trajectories can be constructed
        function modelConfigTrajectoriesTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/modelConfigTrajectoriesTest');
            [mSet] = ModelConfigManager.GetModelConfigListNames();
            for i = 1:length(mSet)
                m = ModelConfig(mSet{i});
                CASPR_log.Debug(['Testing model: ', mSet{i}]);
                trajs_joint = m.getJointTrajectoriesList();
                for j = 1:length(trajs_joint)
                    CASPR_log.Debug(['Testing model: ', mSet{i}, ', trajectory: ', trajs_joint{j}]);
                    traj = m.getJointTrajectory(trajs_joint{j});
                    testCase.assertNotEmpty(traj);
                end
                trajs_op = m.getOperationalTrajectoriesList();
%                 for j = 1:length(trajs_op)
%                     CASPR_log.Debug(['Testing model: ', mSet{i}, ', trajectory: ', trajs_op{j}]);
%                     traj = m.getOperationalTrajectory(trajs_op{j});
%                     testCase.assertNotEmpty(traj);
%                 end
            end
            CASPR_log.Debug('Done ModelConfigTest/modelConfigTrajectoriesTest');            
        end

        % Test that the test master_list is correctly setup
        function testModelListValidTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/testModelListValidTest');
            % Open up and scan through the master list
            fid = fopen([CASPR_configuration.LoadHomePath(),'/data/model_config/test_models/test_models_list.csv']);
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            unique_cell_array = unique(cell_array{1});
            % Two tests are conducted firstly are there any repeated files
            % in the materlist
            assert(length(cell_array{1}) == length(unique_cell_array), 'There are repeated identifiers in the master list');
            CASPR_log.Debug('Done ModelConfigTest/testModelListValidTest');
        end

        % Test that the test model configuration files exist.
        function testModelConfigFilesExistTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/testModelConfigFilesExistTest');
            [mSet] = ModelConfigManager.GetTestModelConfigListNames();
            for i = 1:length(mSet)
                m = TestModelConfig(mSet{i});
                testCase.assertNotEmpty(m);
            end
            CASPR_log.Debug('Done ModelConfigTest/testModelConfigFilesExistTest');
        end

        % Test that all test models can be constructed as bodies.
        function testModelCreationTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/testModelCreationTest');
            [mSet] = ModelConfigManager.GetTestModelConfigListNames();
            for i = 1:length(mSet)
                m = TestModelConfig(mSet{i});
                model = m.getModel(m.defaultCableSetId);
                testCase.assertNotEmpty(model);
            end
            CASPR_log.Debug('Done ModelConfigTest/testModelCreationTest');
        end
        
        % Test that all trajectories can be constructed
        function testModelConfigTrajectoriesTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/testModelConfigTrajectoriesTest');
            [mSet] = ModelConfigManager.GetTestModelConfigListNames();
            for i = 1:length(mSet)
                m = TestModelConfig(mSet{i});
                trajs_joint = m.getJointTrajectoriesList();
                for j = 1:length(trajs_joint)
                    CASPR_log.Debug(['Testing model: ', mSet{i}, ', trajectory: ', trajs_joint{j}]);
                    traj = m.getJointTrajectory(trajs_joint{j});
                    testCase.assertNotEmpty(traj);
                end
                trajs_op = m.getOperationalTrajectoriesList();
                for j = 1:length(trajs_op)
                    CASPR_log.Debug(['Testing model: ', mSet{i}, ', trajectory: ', trajs_op{j}]);
                    traj = m.getOperationalTrajectory(trajs_op{j});
                    testCase.assertNotEmpty(traj);
                end
            end
            CASPR_log.Debug('Done ModelConfigTest/testModelConfigTrajectoriesTest');            
        end

        function devModelListValidTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/devModelListValidTest');
            % Open up and scan through the master list
            fid = fopen([CASPR_configuration.LoadHomePath(),'/data/model_config/indev_models/indev_models_list.csv']);
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            unique_cell_array = unique(cell_array{1});
            % Two tests are conducted firstly are there any repeated files
            % in the materlist
            assert(length(cell_array{1}) == length(unique_cell_array), 'There are repeated identifiers in the master list');
            CASPR_log.Debug('Done ModelConfigTest/devModelListValidTest');
        end

        % Test that the test model configuration files exist.
        function devModelConfigFilesExistTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/devModelConfigFilesExistTest');
            [mSet] = ModelConfigManager.GetDevModelConfigListNames();
            for i = 1:length(mSet)
                CASPR_log.Debug(['Testing model: ', mSet{i}]);
                m = DevModelConfig(mSet{i});
                testCase.assertNotEmpty(m);
            end
            CASPR_log.Debug('Done ModelConfigTest/devModelConfigFilesExistTest');
        end

        % Test that all test models can be constructed as bodies.
        function devModelCreationTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/devModelCreationTest');
            [mSet] = ModelConfigManager.GetDevModelConfigListNames();
            for i = 1:length(mSet)
                CASPR_log.Debug(['Testing model: ', mSet{i}]);
                m = DevModelConfig(mSet{i});
                model = m.getModel(m.defaultCableSetId);
                testCase.assertNotEmpty(model);
            end
            CASPR_log.Debug('Done ModelConfigTest/devModelCreationTest');
        end
        
        % Test that all trajectories can be constructed
        function devModelConfigTrajectoriesTest(testCase)
            CASPR_log.Debug('Running ModelConfigTest/testModelConfigTrajectoriesTest');
            [mSet] = ModelConfigManager.GetDevModelConfigListNames();
            for i = 1:length(mSet)
                m = DevModelConfig(mSet{i});
                CASPR_log.Debug(['Testing model: ', mSet{i}]);
                trajs_joint = m.getJointTrajectoriesList();
                for j = 1:length(trajs_joint)
                    CASPR_log.Debug(['Testing model: ', mSet{i}, ', trajectory: ', trajs_joint{j}]);
                    traj = m.getJointTrajectory(trajs_joint{j});
                    testCase.assertNotEmpty(traj);
                end
                trajs_op = m.getOperationalTrajectoriesList();
%                 for j = 1:length(trajs_op)
%                     CASPR_log.Debug(['Testing model: ', mSet{i}, ', trajectory: ', trajs_op{j}]);
%                     traj = m.getOperationalTrajectory(trajs_op{j});
%                     testCase.assertNotEmpty(traj);
%                 end
            end
            CASPR_log.Debug('Done ModelConfigTest/testModelConfigTrajectoriesTest');            
        end
    end
end
