% Testing of VSD torsion spring 
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelVSDTorsionSpringTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testConstructor(testCase)
            CASPR_log.Debug('Running CableModelVSDTorsionSpringTest/testConstructor');
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            testCase.assertNotEmpty(c);
            CASPR_log.Debug('Done CableModelVSDTorsionSpringTest/testConstructor');
        end
        
        % Test the update function
        function testUpdate(testCase)
            CASPR_log.Debug('Running CableModelVSDTorsionSpringTest/testUpdate');
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            % Create the body model
            model_config = TestModelConfig('test_SCDM');
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel);
            CASPR_log.Debug('Done CableModelVSDTorsionSpringTest/testUpdate');
        end
        
        % Test the length
        function testLength(testCase)
            CASPR_log.Debug('Running CableModelVSDTorsionSpringTest/testLength');
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
            CASPR_log.Debug('Done CableModelVSDTorsionSpringTest/testLength');
        end
        
        % Test the stiffness
        function testK(testCase)
            CASPR_log.Debug('Running CableModelVSDTorsionSpringTest/testK');
            c = CableModelVSDTorsionSpring('1',1,1,1,1,1);
            c.force = 0;
            K = c.K;
            CASPR_log.Debug('Done CableModelVSDTorsionSpringTest/testK');
        end
    end
end