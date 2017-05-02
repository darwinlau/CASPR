% Unit test of the IDObjectiveQuadratic objects
%
% Author        : Felix YUE
% Created       : 2016
% Description    :
classdef IDObjectiveInfNormTest < IDObjectiveTestBase    
    properties (MethodSetupParameter)
        inf_norm_objective_test = {'MinInfCableForce', 'MinInfOptimallySafe'};
    end
    
    methods (TestMethodSetup)
        function setupIDObj(testCase, inf_norm_objective_test)
            if (isequal(inf_norm_objective_test, 'MinInfCableForce'))
                testCase.idObj = IDObjectiveMinInfCableForce(ones(1, testCase.modelObj.numActuators));
            elseif (isequal(inf_norm_objective_test, 'MinInfOptimallySafe'))
                testCase.idObj = IDObjectiveInfOptimallySafe(ones(1, testCase.modelObj.numActuators));
            end
        end
    end
            
    methods (Test)
        function updateObjectiveTest(testCase)
            CASPR_log.Debug('Running IDObjectiveInfNormTest/updateObjectiveTest');
            testCase.idObj.updateObjective(testCase.modelObj);
            testCase.assertSize(testCase.idObj.A, [testCase.modelObj.numActuatorsActive testCase.modelObj.numActuatorsActive], '''A'' matrix is of wrong dimension');
            testCase.assertLength(testCase.idObj.b, testCase.modelObj.numActuatorsActive, '''b'' vector is of wrong dimension');
            CASPR_log.Debug('Done IDObjectiveInfNormTest/updateObjectiveTest');
        end
    end
    
end

