% Test for JointBase.m
%
% Author        : Felix YUE
% Created       : 2016
% Description   :

classdef JointTest < matlab.unittest.TestCase
    properties (ClassSetupParameter)
        joint_type = struct('R_X', JointType.R_X, ...
            'R_Y', JointType.R_Y, ...
            'R_Z', JointType.R_Z, ...
            'U_XY', JointType.U_XY, ...
            'P_XY', JointType.P_XY, ...
            'PLANAR_XY', JointType.PLANAR_XY, ...
            'PLANAR_YZ', JointType.PLANAR_YZ, ...
            'S_EULER_XYZ', JointType.S_EULER_XYZ, ...
            'S_FIXED_XYZ', JointType.S_FIXED_XYZ, ...
            'S_QUATERNION', JointType.S_QUATERNION, ...
            'T_XYZ', JointType.T_XYZ, ...
            'SPATIAL_QUATERNION', JointType.SPATIAL_QUATERNION, ...
            'SPATIAL_EULER_XYZ', JointType.SPATIAL_EULER_XYZ);
        
        
        
    end
    
    properties         
    jointObj;
    end
    
    
    methods(TestClassSetup)
        function createJointObj(testCase, joint_type)
            testCase.jointObj = JointBase.CreateJoint(joint_type);
        end
    end
    
    methods (Test)
        function testUpdate(testCase)
            testCase.jointObj.update(1, 0, 0);
        end
        
        function testDependentVariables(testCase)
            testCase.jointObj.q_deriv;
            testCase.jointObj.S_dot;
            testCase.jointObj.S;
        end
        
    end
end