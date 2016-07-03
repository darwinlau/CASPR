% Base object for testing the IDObjective objects
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef IDObjectiveTestBase < matlab.unittest.TestCase
    properties (MethodSetupParameter)
        model_config_type = struct('IPAnema', ModelConfigType.M_IPANEMA_2, ...
            'Neck_8S', ModelConfigType.M_NECK_8S, ...
            'Active_passive_cables', ModelConfigType.M_PASSIVE_SPRINGS_PLANAR);
    end
    
    properties
        modelObj;
        idObj;
    end
            
    methods (TestMethodSetup)
        function setupModelObj(testCase, model_config_type)
            model_config = ModelConfig(model_config_type);
            testCase.modelObj = model_config.getModel(model_config.defaultCableSetId);
        end
    end
    
end

