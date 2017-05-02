% Testing of VSD flexure 
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelVSDFlexureLinearTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testConstructor(testCase)
            CASPR_log.Debug('Running CableModelVSDFlexureLinearTest/testConstructor');
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            testCase.assertNotEmpty(c);
            CASPR_log.Debug('Done CableModelVSDFlexureLinearTest/testConstructor');
        end
        
        % Test the update function
        function testUpdate(testCase)
            CASPR_log.Debug('Running CableModelVSDFlexureLinearTest/testUpdate');
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            % Create the body model
            model_config = TestModelConfig('test_SCDM');
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
            CASPR_log.Debug('Done CableModelVSDFlexureLinearTest/testUpdate');
        end
        
        % Test the length
        function testLength(testCase)
            CASPR_log.Debug('Running CableModelVSDFlexureLinearTest/testLength');
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
            CASPR_log.Debug('Done CableModelVSDFlexureLinearTest/testLength');
        end
        
        % Test the stiffness
        function testK(testCase)
            CASPR_log.Debug('Running CableModelVSDFlexureLinearTest/testK');
            c = CableModelVSDFlexureLinear('1',1,1,[4,1,2]);
            c.force = 0;
            K = c.K;
            CASPR_log.Debug('Done CableModelVSDFlexureLinearTest/testK');
        end
    end
end