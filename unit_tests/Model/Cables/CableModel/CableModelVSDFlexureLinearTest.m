% Testing of VSD flexure 
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelVSDFlexureLinearTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testCableModelVSDFlexureLinear(testCase)
            disp('Testing CableModelVSDFlexureLinear Constructor')
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            testCase.assertNotEmpty(c);
        end
        
        % Test the update function
        function testUpdate(testCase)
            disp('Testing CableModelVSDFlexureLinear update')
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            % Create the body model
            model_config = ModelConfig(TestModelConfigType.T_SCDM);
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
        end
        
        % Test the length
        function testLength(testCase)
            disp('Testing CableModelVSDFlexureLinear length')
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
        end
        
        % Test the stiffness
        function testK(testCase)
            disp('Testing CableModelVSDFlexureLinear stiffness')
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            c.force = 0;
            K = c.K;
        end
    end
end