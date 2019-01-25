%% Display Loop
repeatBricksNum = 10;
pickOffset = [0.32;0.3;0.205+0.283;0;0;0];
brickHeight = 0.0428;
offsetScene2 = [0.2,0,0.16,0,0,0]';
safePoint = [1;0.3;0.883;0;0;0;];
modelObj.bodyModel.bodies{1}.m = 0.9;
%inverse stacking
for k = 1:repeatBricksNum
    currentLoc = [cell2mat(placeBrickCell(length(placeBrickCell)+1-k));0;0;0;];
    currentang = cell2mat(placeBrickangle(length(placeBrickangle)+1-k));
    
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, currentLoc + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
    myGripper.setServoAngle(currentang);
    pause(0.1);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, currentLoc, time_step, blend_time_default, blend_time_default, v_max));
    pause(1);
    myGripper.setGripperClose;
    myGripper.setKpgain(500);
    modelObj.bodyModel.bodies{1}.m = 2.2;
    pause(0.5);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, currentLoc + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
    pause(0.1);
    pickupPlace = pickOffset+[0;0;(k-1)*brickHeight;0;0;0;];
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, pickupPlace + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
    myGripper.setServoAngle(90);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, pickupPlace, time_step, blend_time_default, blend_time_default, v_max));
    pause(1);
    myGripper.setGripperOpen();
    pause(0.2);   
    modelObj.bodyModel.bodies{1}.m = 0.9;
    myGripper.setKpgain(300);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, pickupPlace + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
end
exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, safePoint, time_step, blend_time_default, blend_time_default, v_max));

%forward build
for l = 1:repeatBricksNum
    pickupPlace = pickOffset+[0;0;(repeatBricksNum - l)*brickHeight;0;0;0;];
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, pickupPlace + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
    myGripper.setServoAngle(90);
    pause(0.1);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, pickupPlace, time_step, blend_time_default, blend_time_default, v_max));
    pause(1);
    myGripper.setGripperClose;
    myGripper.setKpgain(800);
    modelObj.bodyModel.bodies{1}.m = 2.2;
    pause(1);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, pickupPlace + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
%     pause(0.5);
    currentGoal = [cell2mat(placeBrickCell(length(placeBrickCell)-repeatBricksNum+l));0;0;0;];
    currentang = cell2mat(placeBrickangle(length(placeBrickCell)-repeatBricksNum+l));
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, currentGoal + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
    myGripper.setServoAngle(currentang);
    pause(0.1);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, currentGoal, time_step, blend_time_default, blend_time_default, v_max));
    pause(1);
    myGripper.setGripperOpen();
    pause(0.2);   
    modelObj.bodyModel.bodies{1}.m = 0.9;
    myGripper.setKpgain(300);
    exp.runTrajectoryDirectly(JointTrajectory.ParabolicBlendTrajectoryGenerate(exp.q_present, currentGoal + [0;0;0.3;0;0;0], time_step, blend_time_default, blend_time_default, v_max));
    
end