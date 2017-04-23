% Testing of cable model ideal
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef CableModelIdealTest < CableModelTestBase
    methods (Test) 
        % Test the constructor
        function testConstructor(testCase)
            CASPR_log.Debug('Running CableModelIdealTest/testConstructor');
            c = CableModelIdeal('1',1);
            testCase.assertNotEmpty(c)
            CASPR_log.Debug('Done CableModelIdealTest/testConstructor');
        end
        
        % Test the update function
        function testUpdate(testCase)
            CASPR_log.Debug('Running CableModelIdealTest/testUpdate');
            c = CableModelIdeal('1',1);
            % Create the body model
            model_config = TestModelConfig('test_SCDM');
            modelObj = model_config.getModel(model_config.defaultCableSetId);
            c.update(modelObj.bodyModel)
            CASPR_log.Debug('Done CableModelIdealTest/testUpdate');
        end
        
        % Test the length of the springs
        function testLength(testCase)
            CASPR_log.Debug('Running CableModelIdealTest/testLength');
            c = CableModelIdeal('1',1);
            c.force = 0;
            l = c.length;
            testCase.assertPositiveCableLengths(l);
            CASPR_log.Debug('Done CableModelIdealTest/testLength');
        end
        
        % Test the spring stiffness K
        function testK(testCase)
            CASPR_log.Debug('Running CableModelIdealTest/testK');
            c = CableModelIdeal('1',1);
            c.force = 0;
            K = c.K;
            CASPR_log.Debug('Done CableModelIdealTest/testK');
        end
    end
end