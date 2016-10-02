% Testing of linear spring cables
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelLinearSpringTest < CableModelTestBase
    methods (Test) 
        % Test the constructor 
        function testCableModelLinearSpring(testCase)
            disp('Testing CableModelLinearSpring Constructor')
            c = CableModelLinearSpring('1',1);
            testCase.assertNotEmpty(c);
        end
        
        % Test the update function
        function testUpdate(testCase)
            disp('Testing CableModelLinearSpring update')
            c = CableModelLinearSpring('1',1);
            % Create the body model
            model_config = TestModelConfig(TestModelConfigType.T_SCDM);
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
        end
        
        % Test the cable length function
        function testLength(testCase)
            disp('Testing CableModelLinearSpring length')
            c = CableModelLinearSpring('1',1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
        end
        
        % Test the spring stiffness K
        function testK(testCase)
            disp('Testing CableModelLinearSpring stiffness')
            c = CableModelLinearSpring('1',1);
            c.force = 0;
            K = c.K;
        end
    end
end