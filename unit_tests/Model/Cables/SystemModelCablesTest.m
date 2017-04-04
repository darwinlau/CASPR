% Testing of the system model cables
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :
classdef SystemModelCablesTest < matlab.unittest.TestCase
    % Create the model objects for testing
    properties (ClassSetupParameter)
        model_config_type = struct('SCDM', 'test_SCDM', ...
            'MCDM', 'test_MCDM', ...
            'Active_passive_cables', 'test_active_passive_cables', ...
            'HCDM', 'test_HCDM');
    end
    
    properties
        modelObj;
    end
    
    % Create the test case objects from the given model
    methods (TestClassSetup)
        function setupModelObj(testCase, model_config_type)
            model_config = TestModelConfig(model_config_type);
            testCase.modelObj = model_config.getModel(model_config.defaultCableSetId);
        end
    end
    
    methods (Test)        
        % Confirm that the update function works
        function testUpdate(testCase)
            disp('Testing the system Cables update function')
            testCase.modelObj.cableModel.update(testCase.modelObj.bodyModel);
            testCase.assertSystemModelCablePropertySize();
        end
        
        % ----------------------------------------
        % Test the dependent variables
        % ----------------------------------------
        % Test the number of maximum segments
        function testNumSegmentsMax(testCase)
            disp('Testing the number of segments')
            testCase.modelObj.cableModel.numSegmentsMax;              % Maximum number of segments out of all of the cables
        end
        
        % Test the length related variables
        function testLengths(testCase)
            disp('Testing the length variables')
            testCase.modelObj.cableModel.lengths;                     % Vector of lengths for all of the cables
            testCase.modelObj.cableModel.lengthsActive;               % Vector of lengths for active cables 
            testCase.modelObj.cableModel.lengthsPassive;              % Vector of lengths for passive cables
        end
        
        % Test the forces
        function testForces(testCase)
            disp('Testing the force variables')
            testCase.modelObj.cableModel.forces;                      % Vector of forces for all of the cables
            testCase.modelObj.cableModel.forcesActive;                % Vector of forces for active cables
            testCase.modelObj.cableModel.forcesActiveMin;             % Vector of min forces for active cables
            testCase.modelObj.cableModel.forcesActiveMax;             % Vector of max forces for passive cables
            testCase.modelObj.cableModel.forcesPassive;               % Vector of forces for passive cables
        end
        
        % Test the V matrix
        function testVMatrices(testCase)
            disp('Testing the V matrix')
            testCase.modelObj.cableModel.V_active;
            testCase.modelObj.cableModel.V_passive;
        end
    end
    
    methods
        % Confirm that the update produces the correct size
        function assertSystemModelCablePropertySize(testCase)
            testCase.assertSize(testCase.modelObj.cableModel.V,[testCase.modelObj.cableModel.numCables,6*testCase.modelObj.cableModel.numLinks],'V is of the wrong dimension');
            assert(testCase.modelObj.cableModel.numCables == testCase.modelObj.cableModel.numCablesPassive + testCase.modelObj.cableModel.numCablesActive,'Cables must be active or passive');
            testCase.assertSize(testCase.modelObj.cableModel.K,[testCase.modelObj.cableModel.numCables,testCase.modelObj.cableModel.numCables],'K is of the wrong dimension');
        end
    end
end