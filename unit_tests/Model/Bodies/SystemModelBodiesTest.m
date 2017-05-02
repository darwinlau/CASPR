% Testing of the system bodies model cables
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef SystemModelBodiesTest < matlab.unittest.TestCase    
    % Create different models for class
    properties (ClassSetupParameter)
        model_config_type = struct('SCDM', 'test_SCDM', ...
            'MCDM', 'test_MCDM', ...
            'Active_passive_cables', 'test_active_passive_cables', ...
            'HCDM', 'test_HCDM');
    end
    
    properties
        modelObj;
    end
    
    % Set up the test case with the model objects
    methods (TestClassSetup)
        function setupModelObj(testCase, model_config_type)
            model_config = TestModelConfig(model_config_type);
            testCase.modelObj = model_config.getModel(model_config.defaultCableSetId);
        end
    end
    
    methods (Test)
        % Test that the update function works
        function testUpdate(testCase)
            CASPR_log.Debug('Running SystemModelBodiesTest/testUpdate');
            testCase.modelObj.bodyModel.update(zeros(testCase.modelObj.bodyModel.numDofVars,1),zeros(testCase.modelObj.bodyModel.numDofs,1),zeros(testCase.modelObj.bodyModel.numDofs,1),zeros(testCase.modelObj.bodyModel.numDofs,1));
            testCase.assertSystemModelBodiesPropertySize();
            CASPR_log.Debug('Done SystemModelBodiesTest/testUpdate');
        end
        
        % -----------------------------------
        % Test the dependent variables
        % -----------------------------------
        % default values
        function testDefault(testCase)
            CASPR_log.Debug('Running SystemModelBodiesTest/testDefault');
            testCase.modelObj.bodyModel.q_default;
            testCase.assertLength(testCase.modelObj.bodyModel.q_default,testCase.modelObj.bodyModel.numDofVars,'q_default of wrong dimension');
            testCase.modelObj.bodyModel.q_dot_default;
            testCase.assertLength(testCase.modelObj.bodyModel.q_dot_default,testCase.modelObj.bodyModel.numDofs,'q_dot_default of wrong dimension');
            testCase.modelObj.bodyModel.q_ddot_default;
            testCase.assertLength(testCase.modelObj.bodyModel.q_ddot_default,testCase.modelObj.bodyModel.numDofs,'q_ddot_default of wrong dimension');
            CASPR_log.Debug('Done SystemModelBodiesTest/testDefault');
        end
        
        % bounds
        function testQBounds(testCase)
            CASPR_log.Debug('Running SystemModelBodiesTest/testQBounds');
            testCase.modelObj.bodyModel.q_lb;
            testCase.assertLength(testCase.modelObj.bodyModel.q_lb,testCase.modelObj.bodyModel.numDofVars,'q_lb of wrong dimension');
            testCase.modelObj.bodyModel.q_ub;
            testCase.assertLength(testCase.modelObj.bodyModel.q_ub,testCase.modelObj.bodyModel.numDofVars,'q_ub of wrong dimension');
            CASPR_log.Debug('Done SystemModelBodiesTest/testQBounds');
        end
        
        % q_deriv
        function testQDeriv(testCase)
            CASPR_log.Debug('Running SystemModelBodiesTest/testQDeriv');
            testCase.modelObj.bodyModel.q_deriv;
            testCase.assertLength(testCase.modelObj.bodyModel.q_deriv,testCase.modelObj.bodyModel.numDofVars,'q_deriv of wrong dimension');
            CASPR_log.Debug('Done SystemModelBodiesTest/testQDeriv');
        end
    end
    
    methods
        % Confirm that the update has the correct size
        function assertSystemModelBodiesPropertySize(testCase)
            testCase.assertLength(testCase.modelObj.bodyModel.q,testCase.modelObj.bodyModel.numDofVars,'q is of the wrong dimension');
            testCase.assertLength(testCase.modelObj.bodyModel.q_dot,testCase.modelObj.bodyModel.numDofs,'q_dot is of the wrong dimension');
            testCase.assertLength(testCase.modelObj.bodyModel.q_ddot,testCase.modelObj.bodyModel.numDofs,'q_ddot is of the wrong dimension');
            testCase.assertSize(testCase.modelObj.bodyModel.S,[6*testCase.modelObj.bodyModel.numLinks,testCase.modelObj.bodyModel.numDofs],'S is of the wrong dimension');
            testCase.assertSize(testCase.modelObj.bodyModel.S_dot,[6*testCase.modelObj.bodyModel.numLinks,testCase.modelObj.bodyModel.numDofs],'S_dot is of the wrong dimension');
            testCase.assertSize(testCase.modelObj.bodyModel.P,[6*testCase.modelObj.bodyModel.numLinks,6*testCase.modelObj.bodyModel.numLinks],'P is of the wrong dimension');
            testCase.assertSize(testCase.modelObj.bodyModel.W,[6*testCase.modelObj.bodyModel.numLinks,testCase.modelObj.bodyModel.numDofs],'W is of the wrong dimension');
            testCase.assertLength(testCase.modelObj.bodyModel.C_a,6*testCase.modelObj.bodyModel.numLinks,'C_a is of the wrong dimension');
        end
    end
end