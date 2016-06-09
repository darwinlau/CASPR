% Unit test of the model configuration.
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
%    Unit tests to confirm that the model configuration is correct. 
classdef ModelConfigTest < matlab.unittest.TestCase
    methods (Test)
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

