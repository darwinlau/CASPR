% Testing of passive linear springs
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelPassiveLinearSpringTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testCableModelPassiveLinearSpring(testCase)
            disp('Testing CableModelPassiveLinearSpring Constructor')
            c = CableModelPassiveLinearSpring('1',1);
            testCase.assertNotEmpty(c);
        end
        
        % Test the update function
        function testUpdate(testCase)
            disp('Testing CableModelPassiveLinearSpring update')
            c = CableModelPassiveLinearSpring('1',1);
            % Create the body model
            model_config = TestModelConfig('test_SCDM');
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
        end
        
        % Test that the cable length 
        function testLength(testCase)
            disp('Testing CableModelPassiveLinearSpring length')
            c = CableModelPassiveLinearSpring('1',1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
        end
        
        % Test that the spring stiffness K
        function testK(testCase)
            disp('Testing CableModelPassiveLinearSpring stiffness')
            c = CableModelPassiveLinearSpring('1',1);
            c.force = 0;
            K = c.K;
        end
    end
end