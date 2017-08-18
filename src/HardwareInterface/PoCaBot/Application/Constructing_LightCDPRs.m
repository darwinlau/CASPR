%% Application for many bricks
if ~exist('gripper','var')
    gripper = Gripper('COM12');
    gripper.initialize();
end
gripper.setArmAngle(0);
pause(0.5);
gripper.setHandAngle( gripper.LOOSE_HAND_ANGLE);

fo = FileOperation(...
    'C:\Users\user\Documents\GitHub\CASPR_private\src\HardwareInterface\PoCaBot\Application\initstate.ini', ...
    'C:\Users\user\Desktop\project\WorldRoboticsConf_Beijing\DATA FILES\BrickArea_Jason2.csv', ...
    'C:\Users\user\Desktop\project\WorldRoboticsConf_Beijing\DATA FILES\WallDesign_Jason2.csv');
brick_count = fo.getAllBrickCount();
fprintf('There are %d bricks in all.\n',brick_count);

time_step = 0.05;

% where the position of the EE before and after the mission
% 0.417
% Once the working tension is on the cables, the end effector would be lift
% up by 0.02m.
q0 = [2.0 2.0 0.430 0 0 0]';
q_transit_point = [1.98 2.0 0 0 0 0]';

distance_safe = 0.1;
v_max = 0.120; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
blend_time_default = 0.5; %used to decide the acceleration
blend_time_placing = 1.5; %used to decide the deceleration

[pickup_co, place_co]= fo.getCoordinate(fo.readBrickNum());
if ~exist('exp','var')
    exp = PoCaBotExperiment(8,'dualcables_large_endeffector_frame4by4by23_demo',time_step);
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

% The below program is just for debugging. When working for the task, please
% % comment these expressions.
% while(1)  
%     factor = input('The offset constant factor[Nothing means no changing!]:');
%     if ~isempty(factor)
%         exp.factor_offset_per_Newton_Meter = factor;
%     end
%     fprintf('The factor is %0.5f from now on!\n',exp.factor_offset_per_Newton_Meter);
%     q_next = (input('The next q:'))';
%     trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_next, time_step, blend_time_placing, blend_time_placing, v_max);
%     exp.runTrajectoryDirectly(trajectory);
% end

coordinate1 = q_transit_point;
coordinate1(3) = q_temp(3);
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q_temp, coordinate1, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

brick_next_num = fo.readBrickNum();
while ( brick_next_num <= brick_count)
    % read and write file
    brick_present_num = brick_next_num;
    fo.writeBrickNum(brick_present_num+1);
    [pickup_co, place_co]= fo.getCoordinate(brick_present_num);
    [arm_angle_pickup, arm_angle_place] = fo.getArmAngle(brick_present_num);
    
    fprintf('Now working on the %dth brick.',brick_present_num);
    
    gripper.setArmAngle(arm_angle_pickup);
    
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
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate2, coordinate2_pickup, time_step, blend_time_default, blend_time_placing, v_max);
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
    gripper.setArmAngle(arm_angle_place);
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate3, coordinate4, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % place the brick
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate4, coordinate4_placing, time_step, blend_time_default, blend_time_placing, v_max);
    exp.runTrajectoryDirectly(trajectory);
    % move up
    gripper.setHandAngle( gripper.RELEASE_HAND_ANGLE);
    pause(0.5);
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate4_placing, coordinate4, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    gripper.setHandAngle( gripper.LOOSE_HAND_ANGLE);
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
q1 = [1.8 2.0 0.406 0 0 0]';
q1(3) = coordinate3(3);
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(coordinate3, q1, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

% Close the hardware interface
gripper.setArmAngle(0);
%gripper.disconnect();
%exp.application_termination();
%clear exp;
fprintf('Construction Finished.\n');


