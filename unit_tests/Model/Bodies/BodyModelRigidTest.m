% Test class for the rigid body model
%
% Author        : Felix YUE
% Created       : 2016
% Description    :

classdef BodyModelRigidTest < matlab.unittest.TestCase    
    methods (Test)
        % Test that the constructor works
        function testBodyModelRigid(testCase)
            CASPR_log.Debug('Running BodyModelRigidTest/testBodyModelRigid');
            j = JointBase.CreateJoint(JointType.R_X,0);
            bk = BodyModelRigid(1, 'Body1', j);
            CASPR_log.Debug('Done BodyModelRigidTest/testBodyModelRigid');
        end

        % Confirm that the rigid body can be updated.
        function testUpdate(testCase)
            CASPR_log.Debug('Running BodyModelRigidTest/testUpdate');
            j = JointBase.CreateJoint(JointType.R_X,0);
            bk = BodyModelRigid(1, 'Body1', j);
            bk.update(0, 0, 0);
            CASPR_log.Debug('Done BodyModelRigidTest/testUpdate');
        end
    end
end