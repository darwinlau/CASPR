classdef ModelConfigTest < matlab.unittest.TestCase
    %MODELCONFIGTEST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Test)
        function modelConfigFilesExist(testCase)
            disp('modelConfigFilesExist test');
            [mSet, mNames] = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                disp(['Testing ModelConfigType: ', mNames{i}]);
                m = ModelConfig(mSet(i));
                testCase.assertNotEmpty(m);
            end
        end
        
        function modelJointCreation(testCase)
            disp('modelJointCreation test');
            [mSet, mNames] = enumeration('JointType');
            for i = 1:length(mSet)
                disp(['Testing JointType: ', mNames{i}]);
                j = Joint.CreateJoint(mSet(i));
                testCase.assertNotEmpty(j);
            end
        end
        
        function modelBodyCreation(testCase)
            disp('modelBodyCreation test');
            [mSet, mNames] = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                disp(['Testing ModelConfigType: ', mNames{i}]);
                m = ModelConfig(mSet(i));
                body_xmlobj = m.getBodiesPropertiesXmlObj();
                bodyModel = SystemModelBodies.LoadXmlObj(body_xmlobj);
                testCase.assertNotEmpty(bodyModel);
            end
        end
    end
    
end

