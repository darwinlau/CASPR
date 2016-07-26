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
                testCase.idObj = IDObjectiveMinQuadCableForce(ones(1, testCase.modelObj.numCables));
            elseif (isequal(quad_objective_test, 'MinInteractions'))
                testCase.idObj = IDObjectiveMinInteractions(ones(1, 6*testCase.modelObj.numLinks));
            elseif (isequal(quad_objective_test, 'QuadOptimallySafe'))
                testCase.idObj = IDObjectiveQuadOptimallySafe(ones(1, testCase.modelObj.numCables));
            end
        end
    end
            
    methods (Test)
        function updateObjectiveTest(testCase)
            disp('Testing quadratic objective update')
            testCase.idObj.updateObjective(testCase.modelObj);
            testCase.assertSize(testCase.idObj.A, [testCase.modelObj.numCablesActive testCase.modelObj.numCablesActive], '''A'' matrix is of wrong dimension');
            testCase.assertLength(testCase.idObj.b, testCase.modelObj.numCablesActive, '''b'' vector is of wrong dimension');
        end
    end
    
end

