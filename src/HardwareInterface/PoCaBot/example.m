clc;clear;

%% sample codes for the application of the Gripper class
% gripper = Gripper('COM6');
% gripper.initialize();
% MAX_HAND_ANGLE = 180;
% MIN_HAND_ANGLE = 70;
% BEST_HAND_ANGLE = 96;
% MAX_ARM_ANGLE  = 180;
% MIN_ARM_ANGLE  = 0;
% gripper.setHandAngle( MIN_HAND_ANGLE);
% gripper.setHandAngle( MAX_HAND_ANGLE);
% gripper.setHandArm_Angle( MIN_HAND_ANGLE,MIN_ARM_ANGLE);
% distance = gripper.getUltraSonic();
% gripper.setUS_ENABLE(true);
% distance = gripper.getUltraSonic();
% gripper.disconnect();

%% sample codes for the applicaiton of PoCaBotExperiment class
% exp = PoCaBotExperiment(8);
% exp.motorTest();

%% simple trajectory
trajectory_id = 'simple_move_brick';
%             trajectory_id = 'up';
%             trajectory_id = 'down';
%             trajectory_id = '3DPrinting';
exp = PoCaBotExperiment();
trajectory = exp.modelConfig.getTrajectory(trajectory_id);
exp.runTrajectory(trajectory);

figure;
plot(trajectory.timeVector, exp.l_feedback_traj,'*'); hold on;
legend('1','2','3','4','5','6','7','8','Location','NorthEast');
plot(trajectory.timeVector, exp.l_cmd_traj);
% exp.l_cmd_traj(:,1)
%    figure;

