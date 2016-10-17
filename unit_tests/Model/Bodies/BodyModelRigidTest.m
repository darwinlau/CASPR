% Test class for the rigid body model
%
% Author        : Felix YUE
% Created       : 2016
% Description    :

classdef BodyModelRigidTest < matlab.unittest.TestCase    
    methods (Test)
        % Test that the constructor works
        function testBodyModelRigid(testCase)
            disp('Testing BodyModelRigid Constructor')
            j = JointBase.CreateJoint(JointType.R_X,0);
            bk = BodyModelRigid(1, 'Body1', j);
        end

         % Confirm that the rigid body can be updated.
         function testUpdate(testCase)
             disp('Testing BodyModelRigid update')
             j = JointBase.CreateJoint(JointType.R_X,0);
             bk = BodyModelRigid(1, 'Body1', j);
             bk.update(0, 0, 0)
         end
    end
end