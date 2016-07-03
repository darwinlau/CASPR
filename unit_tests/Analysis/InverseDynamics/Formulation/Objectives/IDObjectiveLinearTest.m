% Unit test of the IDObjectiveQuadratic objects
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :
classdef IDObjectiveLinearTest < IDObjectiveTestBase    
    properties (MethodSetupParameter)
        lin_objective_test = {'MinLinCableForce'};
    end
    
    methods (TestMethodSetup)
        function setupIDObj(testCase, lin_objective_test)
            if (isequal(lin_objective_test, 'MinLinCableForce'))
                testCase.idObj = IDObjectiveMinLinCableForce(ones(1, testCase.modelObj.numCables));
            end
        end
    end
        
    methods (Test)
        function updateObjectiveTest(testCase)
            testCase.idObj.updateObjective(testCase.modelObj);
            testCase.assertTrue(length(testCase.idObj.b) == testCase.modelObj.numCablesActive, '''b'' vector is of wrong dimension');
        end
    end
    
end

