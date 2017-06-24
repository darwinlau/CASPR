% clc;% clear;

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
% gripper.setArmAngle(180);
% 
% distance = gripper.getUltraSonic();
% gripper.setUS_ENABLE(true);
% distance = gripper.getUltraSonic();
% gripper.disconnect();

%% sample codes for the applicaiton of PoCaBotExperiment class
% exp = PoCaBotExperiment(1);
% exp.motorTest();

%% sample codes for the file operation
% fo = FileOperation(...
%     'C:\Users\Tristan\Desktop\Constructing Demo\initstate.ini', ...
%     'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\brickLayout.csv', ...
%     'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\wallDesign.csv');
% brick_count = fo.getAllBrickCount();
% fo.readInitPos_Motors()
% fo.readBrickNum()
% fo.writeInitPos_Motors([8:-1:1]');
% fo.readInitPos_Motors()
% fo.readBrickNum()
% 
% num = fo.readBrickNum();% or num = 1 if at the beginning of the constructing.
% for index = num:brick_count
%     
%     [pickup_co, place_co]= fo.getCoordinate(index);
%     fo.writeBrickNum(index);
%     fo.readBrickNum()
% end

%% simple trajectory
% trajectory_id = 'simple_move_brick_on_larger_scale_frame';
% %             trajectory_id = 'up';
% %             trajectory_id = 'down';
% %             trajectory_id = '3DPrinting';
% exp = PoCaBotExperiment();
% trajectory = exp.modelConfig.getJointTrajectory(trajectory_id);
% exp.runTrajectory(trajectory);
% 
% figure;
% plot(trajectory.timeVector, exp.l_feedback_traj,'*'); hold on;
% legend('1','2','3','4','5','6','7','8','Location','NorthEast');
% plot(trajectory.timeVector, exp.l_cmd_traj);
% % exp.l_cmd_traj(:,1)
% %    figure;
% 
% % Close the hardware interface
% % exp.hardwareInterface.systemOffSend();
% % exp.closeHardwareInterface();
% % exp.gripper.disconnect();

%% Test the method 'generateTrajectoryParabolicBlend' of Class PoCaBotExperiment
% q_s = zeros(3,1);
% q_e = [0.0;0.0;0.2];
% time_step = 0.05;
% time_blend_s = 0.1;
% time_blend_e = 1;
% v_max = 0.167;
% [trajectory] = PoCaBotExperiment.generateTrajectoryParabolicBlend(q_s, q_e, time_step, time_blend_s, time_blend_e, v_max);
% plot(trajectory.timeVector,trajectory.q);
% plot(trajectory.timeVector,trajectory.q_dot);
% plot(trajectory.timeVector,trajectory.q_ddot);

%% Application for many bricks
if ~exist('gripper','var')
    gripper = Gripper('COM6');
    gripper.initialize();
end
gripper.setArmAngle(0);
pause(0.5);
gripper.setHandAngle( gripper.LOOSE_HAND_ANGLE);

fo = FileOperation(...
    'C:\Users\Tristan\Desktop\Constructing Demo\initstate.ini', ...
    'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\WallDesign_Jason1.csv', ...
    'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\BrickArea_Jason1.csv');
brick_count = fo.getAllBrickCount();
fprintf('There are %d bricks in all.\n',brick_count);

time_step = 0.05;

% where the position of the EE before and after the mission
q0 = [1.140 1.140 0.406 0 0 0]';
q_transit_point = [1.30 1.140 0 0 0 0]';

distance_safe = 0.1;
v_max = 0.145; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
blend_time_default = 0.5; %used to decide the acceleration
blend_time_placing = 1.5; %used to decide the deceleration

[pickup_co, place_co]= fo.getCoordinate(fo.readBrickNum());
if ~exist('exp','var')
    exp = PoCaBotExperiment();
    exp.application_preparation(fo,q0);
    q_temp = q0;
    q_temp(3) = distance_safe+max(pickup_co(3),place_co(3));
    % lift the gripper from the ground
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q_temp, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
else
    q_temp = exp.q_present;
    q_temp(3) = distance_safe+max(pickup_co(3),place_co(3));
    % lift the gripper to prevent collision
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_temp, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
end

coordinate1 = q_transit_point;
coordinate1(3) = q_temp(3);
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q_temp, coordinate1, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

% current = ones(8,1)*500;
% exp.hardwareInterface.currentCommandSend(current);

brick_next_num = fo.readBrickNum();
while ( brick_next_num <= brick_count)
    % read and write file
    brick_present_num = brick_next_num;
    fo.writeBrickNum(brick_present_num+1);
    [pickup_co, place_co]= fo.getCoordinate(brick_present_num);
    [arm_angle_pickup, arm_angle_place] = fo.getArmAngle(brick_present_num);
    
    fprintf('Now working on the %dth brick.',brick_present_num);
    
    % load coordinate 2 bar
    coordinate2_pickup = [pickup_co;zeros(3,1)];
    % load coordinate 4 bar
    coordinate4_placing = [place_co;zeros(3,1)];
    
    % load coordinate 2
    coordinate2 = coordinate2_pickup;
    coordinate2(3) = coordinate2_pickup(3) + distance_safe;
    % load coordinate 3 which is the central point btween the construction
    % and the bricks.
    coordinate3 = q_transit_point;
    coordinate3(3) = max(coordinate4_placing(3), coordinate2_pickup(3))+ distance_safe;
    % load coordinate 4
    coordinate4 = coordinate4_placing;
    coordinate4(3) = coordinate4_placing(3) + distance_safe;

    % move to the point upon the pickup point
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate1, coordinate2, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % lower the end effector to pick up the brick
    gripper.setArmAngle(arm_angle_pickup);
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate2, coordinate2_pickup, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % lift the brick
    gripper.setHandAngle( gripper.BEST_HAND_ANGLE);
    pause(0.5);
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate2_pickup, coordinate2, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % move to the central point
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate2, coordinate3, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % move to the point uprightly upon the placing point
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate3, coordinate4, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % place the brick
    gripper.setArmAngle(arm_angle_place);
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate4, coordinate4_placing, time_step, blend_time_default, blend_time_placing, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % move up
    gripper.setHandAngle( gripper.LOOSE_HAND_ANGLE);
    pause(0.5);
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate4_placing, coordinate4, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % move to the central point
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate4, coordinate3, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    
    fprintf('   Successfully finished.\n');
    
    coordinate1 = coordinate3;
    % figure;
    % plot(trajectory.timeVector, exp.l_feedback_traj,'*'); hold on;
    % legend('1','2','3','4','5','6','7','8','Location','NorthEast');
    % plot(trajectory.timeVector, exp.l_cmd_traj);
    
    brick_next_num = fo.readBrickNum();
end
q1 = [1.000 1.140 0.406 0 0 0]';
q1(3) = coordinate3(3);
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate3, q1, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

% Close the hardware interface
gripper.setArmAngle(0);
%gripper.disconnect();
%exp.application_termination();
%clear exp;
fprintf('Construction Finished.\n');


