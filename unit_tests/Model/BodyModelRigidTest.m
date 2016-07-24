% System kinematics and dynamics of a rigid body unit test
%
% Author        : Darwin LAU
% Created       : 2016
% Description    :

classdef BodyModelRigidTest < matlab.unittest.TestCase    
    methods (Test)
        % The constructor
        function TestBodyModelRigid(testCase)
            j = JointBase.CreateJoint(JointType.R_X);
            bk = BodyModelRigid(1, 'Body1', j);
        end

         % Updates the joint space
         
         function testUpdate(testCase)
             j = JointBase.CreateJoint(JointType.R_X);
             bk = BodyModelRigid(1, 'Body1', j);
             bk.update(0, 0, 0)
         end
    end

    
    
end