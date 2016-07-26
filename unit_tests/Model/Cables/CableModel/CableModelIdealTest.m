% Testing of cable model ideal
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelIdealTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testCableModelIdeal(testCase)
            disp('Testing CableModelIdeal Constructor')
            c = CableModelIdeal('1',1);
            testCase.assertNotEmpty(c)
        end
        
        % Test the update function
        function testUpdate(testCase)
            disp('Testing CableModelIdeal update')
            c = CableModelIdeal('1',1);
            % Create the body model
            model_config = ModelConfig(TestModelConfigType.T_SCDM);
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
        end
        
        % Test the length of the springs
        function testLength(testCase)
            disp('Testing CableModelIdeal length')
            c = CableModelIdeal('1',1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
        end
        
        % Test the spring stiffness K
        function testK(testCase)
            disp('Testing CableModelIdeal stiffness')
            c = CableModelIdeal('1',1);
            c.force = 0;
            K = c.K;
        end
    end
end