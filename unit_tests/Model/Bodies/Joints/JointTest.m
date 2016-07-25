% Test for JointBase.m
%
% Author        : Felix YUE
% Created       : 2016
% Description   :

classdef JointTest < matlab.unittest.TestCase
    % Compile a test structure corresponding of all joint types.
    properties (ClassSetupParameter)
        joint_type = TestingOperations.createTestClassSetupParameter('JointType');
    end

    % Test case properties
    properties
        jointObj;
    end

    % Create the jointObj properties.
    methods(TestClassSetup)
        function createJointObj(testCase, joint_type)
            testCase.jointObj = JointBase.CreateJoint(joint_type);
        end
    end

    methods (Test)
        % Test that the joint can be updated
        function testUpdate(testCase)
            switch(testCase.jointObj.type)
                case JointType.S_QUATERNION
                    testCase.jointObj.update([1;0;0;0], zeros(testCase.jointObj.numDofs,1), zeros(testCase.jointObj.numDofs,1));
                case JointType.SPATIAL_QUATERNION
                    testCase.jointObj.update([0;0;0;1;0;0;0], zeros(testCase.jointObj.numDofs,1), zeros(testCase.jointObj.numDofs,1));
                otherwise
                    testCase.jointObj.update(zeros(testCase.jointObj.numVars,1), zeros(testCase.jointObj.numDofs,1), zeros(testCase.jointObj.numDofs,1));
            end
            testCase.assertJointPropertySize()
        end

        % ---------------------------------
        % Tests for the dependent variables
        % ---------------------------------
        % First q_deriv
        function testQDerive(testCase)
            testCase.jointObj.q_deriv;
            testCase.assertLength(testCase.jointObj.q_deriv,testCase.jointObj.numVars,'q_deriv of wrong dimension');
        end
        
        % First S_dot
        function testSDot(testCase)
            testCase.jointObj.S_dot;
            testCase.assertSize(testCase.jointObj.S_dot,[6,testCase.jointObj.numDofs],'S_dot of wrong dimension');
        end
        
        % First S
        function testS(testCase)
            testCase.jointObj.S;
            testCase.assertSize(testCase.jointObj.S,[6,testCase.jointObj.numDofs],'S of wrong dimension');
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
