% Testing of passive linear springs
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelPassiveLinearSpringTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testCableModelPassiveLinearSpring(testCase)
            c = CableModelPassiveLinearSpring('1',1);
            testCase.assertNotEmpty(c);
        end
        
        % Test the update function
        function testUpdate(testCase)
            c = CableModelPassiveLinearSpring('1',1);
            % Create the body model
            model_config = ModelConfig(TestModelConfigType.T_SCDM);
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
        end
        
        % Test that the cable length 
        function testLength(testCase)
            c = CableModelPassiveLinearSpring('1',1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
        end
        
        % Test that the spring stiffness K
        function testK(testCase)
            c = CableModelPassiveLinearSpring('1',1);
            c.force = 0;
            K = c.K;
        end
    end
end