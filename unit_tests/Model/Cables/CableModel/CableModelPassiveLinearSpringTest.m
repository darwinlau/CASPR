% Testing of passive linear springs
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelPassiveLinearSpringTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testConstructor(testCase)
            CASPR_log.Debug('Running CableModelPassiveLinearSpringTest/testConstructor');
            c = CableModelPassiveLinearSpring('1',1);
            testCase.assertNotEmpty(c);
            CASPR_log.Debug('Done CableModelPassiveLinearSpringTest/testConstructor');
        end
        
        % Test the update function
        function testUpdate(testCase)
            CASPR_log.Debug('Running CableModelPassiveLinearSpringTest/testUpdate');
            c = CableModelPassiveLinearSpring('1',1);
            % Create the body model
            model_config = TestModelConfig('test_SCDM');
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
            CASPR_log.Debug('Done CableModelPassiveLinearSpringTest/testUpdate');
        end
        
        % Test that the cable length 
        function testLength(testCase)
            CASPR_log.Debug('Running CableModelPassiveLinearSpringTest/testLength');
            c = CableModelPassiveLinearSpring('1',1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
            CASPR_log.Debug('Done CableModelPassiveLinearSpringTest/testLength');
        end
        
        % Test that the spring stiffness K
        function testK(testCase)
            CASPR_log.Debug('Running CableModelPassiveLinearSpringTest/testK');
            c = CableModelPassiveLinearSpring('1',1);
            c.force = 0;
            K = c.K;
            CASPR_log.Debug('Done CableModelPassiveLinearSpringTest/testK');
        end
    end
end