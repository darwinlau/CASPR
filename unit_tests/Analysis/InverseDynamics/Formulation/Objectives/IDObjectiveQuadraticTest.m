% Unit test of the IDObjectiveQuadratic objects
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef IDObjectiveQuadraticTest < IDObjectiveTestBase    
    properties (MethodSetupParameter)
        quad_objective_test = {'MinQuadCableForce', 'MinInteractions', 'QuadOptimallySafe'};
    end
    
    methods (TestMethodSetup)
        function setupIDObj(testCase, quad_objective_test)
            if (isequal(quad_objective_test, 'MinQuadCableForce'))
                testCase.idObj = IDObjectiveMinQuadCableForce(ones(1, testCase.modelObj.numActuators));
            elseif (isequal(quad_objective_test, 'MinInteractions'))
                testCase.idObj = IDObjectiveMinInteractions(ones(1, 6*testCase.modelObj.numLinks));
            elseif (isequal(quad_objective_test, 'QuadOptimallySafe'))
                testCase.idObj = IDObjectiveQuadOptimallySafe(ones(1, testCase.modelObj.numActuators));
            end
        end
    end
            
    methods (Test)
        function updateObjectiveTest(testCase)
            CASPR_log.Debug('Running IDObjectiveQuadraticTest/updateObjectiveTest');
            testCase.idObj.updateObjective(testCase.modelObj);
            testCase.assertSize(testCase.idObj.A, [testCase.modelObj.numActuatorsActive testCase.modelObj.numActuatorsActive], '''A'' matrix is of wrong dimension');
            testCase.assertLength(testCase.idObj.b, testCase.modelObj.numActuatorsActive, '''b'' vector is of wrong dimension');
            CASPR_log.Debug('Done IDObjectiveQuadraticTest/updateObjectiveTest');
        end
    end
    
end

