% Base object for testing the IDObjective objects
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef IDObjectiveTestBase < matlab.unittest.TestCase
    properties (ClassSetupParameter)
        model_config_type = struct('SCDM', 'test_SCDM', ...
            'MCDM', 'test_MCDM', ...
            'Active_passive_cables', 'test_active_passive_cables', ...
            'HCDM', 'test_HCDM');
    end
    
    properties
        modelObj;
        idObj;
    end
            
    methods (TestClassSetup)
        function setupModelObj(testCase, model_config_type)
            model_config = TestModelConfig(model_config_type);
            testCase.modelObj = model_config.getModel(model_config.defaultCableSetId);
        end
    end
    
end