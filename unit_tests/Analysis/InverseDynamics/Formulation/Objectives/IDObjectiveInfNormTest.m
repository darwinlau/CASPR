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
                testCase.idObj = IDObjectiveMinInfCableForce(ones(1, testCase.modelObj.numCables));
            elseif (isequal(inf_norm_objective_test, 'MinInfOptimallySafe'))
                testCase.idObj = IDObjectiveInfOptimallySafe(ones(1, testCase.modelObj.numCables));
            end
        end
    end
            
    methods (Test)
        function updateObjectiveTest(testCase)
            disp('Testing inf norm objective update')
            testCase.idObj.updateObjective(testCase.modelObj);
            testCase.assertSize(testCase.idObj.A, [testCase.modelObj.numCablesActive testCase.modelObj.numCablesActive], '''A'' matrix is of wrong dimension');
            testCase.assertLength(testCase.idObj.b, testCase.modelObj.numCablesActive, '''b'' vector is of wrong dimension');
        end
    end
    
end

