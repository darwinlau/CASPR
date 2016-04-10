classdef ModelConfigTest < matlab.unittest.TestCase
    %MODELCONFIGTEST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Test)
        function checkModelConfigFilesExist(testCase)
            [mSet, mNames] = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                disp(['Checking ModelConfigType: ', mNames{i}]);
                m = ModelConfig(mSet(i));
            end
            testCase.assertNotEmpty(m);
        end
    end
    
end

