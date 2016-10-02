% Testing of VSD torsion spring 
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelVSDTorsionSpringTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testCableModelVSDTorsionSpring(testCase)
            disp('Testing CableModelVSDTorsionSpring Constructor')
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            testCase.assertNotEmpty(c);
        end
        
        % Test the update function
        function testUpdate(testCase)
            disp('Testing CableModelVSDTorsionSpring update')
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            % Create the body model
            model_config = TestModelConfig(TestModelConfigType.T_SCDM);
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel);
        end
        
        % Test the length
        function testLength(testCase)
            disp('Testing CableModelVSDTorsionSpring length')
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
        end
        
        % Test the stiffness
        function testK(testCase)
            disp('Testing CableModelVSDTorsionSpring stiffness')
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            c.force = 0;
            K = c.K;
        end
    end
end