% Unit test of the model configuration.
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    Unit tests to confirm that the model configuration is correct. 
classdef ModelConfigTest < matlab.unittest.TestCase
    methods (Test)
        % Test that the master_list is correctly setup
        function masterListValid(testCase)
            disp('masterListValid test');
            load('CASPR_environment.mat','home_path');
            fid = fopen([home_path,'/data/config/models/master_list.csv']);
            cell_array = textscan(fid,'%s %s %s %s %s %s','delimiter',',');
            unique_cell_array = unique(cell_array{1});
            % Two tests are conducted firstly are there any repeated files
            % in the materlist
            assert(length(cell_array{1}) == length(unique_cell_array),'There are repeated identifiers in the master list');
            % Secondly does each iterm in the list map to an enum.
            for i = 1:length(cell_array{1})
                % Check if the enum exists
                ModelConfigType.(cell_array{1}{i});
            end
        end
        
        % Test that the model configuration files exist.
        function modelConfigFilesExist(testCase)
            disp('modelConfigFilesExist test');
            [mSet, mNames] = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                disp(['Testing ModelConfigType: ', mNames{i}]);
                m = ModelConfig(mSet(i));
                testCase.assertNotEmpty(m);
            end
        end
        
        % Test that all defined joints can be instantiated.
        function modelJointCreation(testCase)
            disp('modelJointCreation test');
            [mSet, mNames] = enumeration('JointType');
            for i = 1:length(mSet)
                disp(['Testing JointType: ', mNames{i}]);
                j = Joint.CreateJoint(mSet(i));
                testCase.assertNotEmpty(j);
            end
        end
        
        % Test that all models can be constructed as bodies.
        function modelCreation(testCase)
            disp('modelBodyCreation test');
            [mSet, mNames] = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                disp(['Testing ModelConfigType: ', mNames{i}]);
                m = ModelConfig(mSet(i));
                model = m.getModel(m.defaultCableSetId);
                testCase.assertNotEmpty(model);
            end
        end
    end
    
end

