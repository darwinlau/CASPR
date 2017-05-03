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
                testCase.idObj = IDObjectiveMinLinCableForce(ones(1, testCase.modelObj.numActuators));
            end
        end
    end
        
    methods (Test)
        function updateObjectiveTest(testCase)
            CASPR_log.Debug('Running IDObjectiveLinearTest/updateObjectiveTest');
            testCase.idObj.updateObjective(testCase.modelObj);
            testCase.assertLength(testCase.idObj.b, testCase.modelObj.numActuatorsActive, '''b'' vector is of wrong dimension');
            CASPR_log.Debug('Done IDObjectiveLinearTest/updateObjectiveTest');
        end
    end
    
end

