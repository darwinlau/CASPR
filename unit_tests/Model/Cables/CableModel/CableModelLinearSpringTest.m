% Testing of linear spring cables
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelLinearSpringTest < CableModelTestBase
    methods (Test) 
        % Test the constructor 
        function testConstructor(testCase)
            CASPR_log.Debug('Running CableModelLinearSpringTest/testConstructor');
            c = CableModelLinearSpring('1',1);
            testCase.assertNotEmpty(c);
            CASPR_log.Debug('Done CableModelLinearSpringTest/testConstructor');
        end
        
        % Test the update function
        function testUpdate(testCase)
            CASPR_log.Debug('Running CableModelLinearSpringTest/testUpdate');
            c = CableModelLinearSpring('1',1);
            % Create the body model
            model_config = TestModelConfig('test_SCDM');
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
            CASPR_log.Debug('Done CableModelLinearSpringTest/testUpdate');
        end
        
        % Test the cable length function
        function testLength(testCase)
            CASPR_log.Debug('Running CableModelLinearSpringTest/testLength');
            c = CableModelLinearSpring('1',1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
            CASPR_log.Debug('Done CableModelLinearSpringTest/testLength');
        end
        
        % Test the spring stiffness K
        function testK(testCase)
            CASPR_log.Debug('Running CableModelLinearSpringTest/testK');
            c = CableModelLinearSpring('1',1);
            c.force = 0;
            K = c.K;
            CASPR_log.Debug('Done CableModelLinearSpringTest/testK');
        end
    end
end