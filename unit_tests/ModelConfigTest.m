classdef ModelConfigTest < matlab.unittest.TestCase
    %MODELCONFIGTEST Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods (Test)
        function checkModelConfigFiles(testCase)
            mSet = enumeration('ModelConfigType');
            for i = 1:length(mSet)
                m = ModelConfig(mSet(i));
            end
            testCase.assertNotEmpty(m);
        end
    end
    
end

