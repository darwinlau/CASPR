% Test for JointBase.m
%
% Author        : Felix YUE
% Created       : 2016
% Description   :
classdef JointTest < matlab.unittest.TestCase
    % Compile a test structure corresponding of all joint types.
    properties (ClassSetupParameter)
        joint_type = UnitTestOperations.createTestClassSetupParameter('JointType');
        % unit test operations
    end

    % Test case properties
    properties
        jointObj;
    end

    % Create the jointObj properties.
    methods(TestClassSetup)
        function createJointObj(testCase, joint_type)
            if (joint_type == JointType.R_AXIS || joint_type == JointType.P_AXIS)
                testCase.jointObj = JointBase.CreateJoint(joint_type, [], [], [], 0, [1; 0; 0]);
            else
                testCase.jointObj = JointBase.CreateJoint(joint_type, [], [], [], 0);
            end
        end
    end

    methods (Test)
        % Test that the joint can be updated
        function testUpdate(testCase)
            CASPR_log.Debug('Running JointTest/testUpdate');
            testCase.jointObj.update(testCase.jointObj.q_initial, zeros(testCase.jointObj.numDofs,1), zeros(testCase.jointObj.numDofs,1));
            testCase.assertJointPropertySize();
            CASPR_log.Debug('Done JointTest/testUpdate');
        end

        % ---------------------------------
        % Tests for the dependent variables
        % ---------------------------------
        % First q_deriv
        function testQDeriv(testCase)
            CASPR_log.Debug('Running JointTest/testQDeriv');
            testCase.jointObj.q_deriv;
            testCase.assertLength(testCase.jointObj.q_deriv,testCase.jointObj.numVars,'q_deriv of wrong dimension');
            CASPR_log.Debug('Done JointTest/testQDeriv');
        end
        
        % First S_dot
        function testSDot(testCase)
            CASPR_log.Debug('Running JointTest/testSDot');
            testCase.jointObj.S_dot;
            testCase.assertSize(testCase.jointObj.S_dot,[6,testCase.jointObj.numDofs],'S_dot of wrong dimension');
            CASPR_log.Debug('Done JointTest/testSDot');
        end
        
        % First S
        function testS(testCase)
            CASPR_log.Debug('Running JointTest/testS');
            testCase.jointObj.S;
            testCase.assertSize(testCase.jointObj.S,[6,testCase.jointObj.numDofs],'S of wrong dimension');
            CASPR_log.Debug('Done JointTest/testS');
        end
    end
    
    methods 
        % Confirm that the resulting joint information is of the correct
        % size.
        function assertJointPropertySize(testCase)
            % The joint position information
            testCase.assertLength(testCase.jointObj.q,testCase.jointObj.numVars,'q of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_dot,testCase.jointObj.numDofs,'q_dot of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_ddot,testCase.jointObj.numDofs,'q_ddot of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_initial,testCase.jointObj.numVars,'q_initial of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_default,testCase.jointObj.numVars,'q_default of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_dot_default,testCase.jointObj.numDofs,'q_dot_default of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_ddot_default,testCase.jointObj.numDofs,'q_ddot_default of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_lb,testCase.jointObj.numVars,'q_lb of wrong dimension');
            testCase.assertLength(testCase.jointObj.q_ub,testCase.jointObj.numVars,'q_ub of wrong dimension');
            % Relative position information
            testCase.assertSize(testCase.jointObj.R_pe,[3,3],'R_pe is of the wrong dimension');
            testCase.assertLength(testCase.jointObj.r_rel,3,'r_rel of wrong dimension');
            if(testCase.jointObj.numDofs == 1)
                testCase.assertSize(testCase.jointObj.S_grad,[6,testCase.jointObj.numDofs],'S_grad is of the wrong dimension');
                testCase.assertSize(testCase.jointObj.S_dot_grad,[6,testCase.jointObj.numDofs],'S_dot_grad is of the wrong dimension');
            else
                testCase.assertSize(testCase.jointObj.S_grad,[6,testCase.jointObj.numDofs,testCase.jointObj.numDofs],'S_grad is of the wrong dimension');
                testCase.assertSize(testCase.jointObj.S_dot_grad,[6,testCase.jointObj.numDofs,testCase.jointObj.numDofs],'S_dot_grad is of the wrong dimension');
            end
        end
    end
end
