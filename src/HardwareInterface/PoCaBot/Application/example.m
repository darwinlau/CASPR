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

%% ID TEST
% model_config = ModelConfig('PoCaBot spatial');
% cable_set_id = 'dualcables_large_endeffector_frame4by4by4_demo';
% if ~exist('idsim','var')
%     % Load the SystemKinematics object from the XML
%     modelObj = model_config.getModel(cable_set_id);
%     % Determine the solver
%     id_objective = IDObjectiveMinLinCableForce(ones(modelObj.numActuatorsActive,1));
%     id_solver = IDSolverLinProg(modelObj, id_objective, ID_LP_SolverType.MATLAB);
%     idsim = InverseDynamicsSimulator(modelObj, id_solver);
% end
% 
% q0 = [1;1;1;0;0;0];
% q1 = q0;
% q1(1) = q0(1)+1;
% time_step = 0.05;
% blend_time_default = 0.5;
% v_max = 0.01;
% trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q1, time_step, blend_time_default, blend_time_default, v_max);
% tic
% for t = 1:length(trajectory.timeVector)
%     [~, model, ~, ~, ~] = idsim.IDSolver.resolve(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t), zeros(idsim.model.numDofs,1));
%     cableForces = model.cableForces;
%     cableLengths = model.cableLengths;
% end
% duration = toc;
% fprintf('The avarage time for each update is %0.3fs.\n',duration/length(trajectory.timeVector));





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
% q_e = [0.0;0.0;0.0];
% time_step = 0.05;
% time_blend_s = 0;
% time_blend_e = 0;
% v_max = 0.167;
% [trajectory] = PoCaBotExperiment.generateTrajectoryParabolicBlend(q_s, q_e, time_step, time_blend_s, time_blend_e, v_max);
% plot(trajectory.timeVector,trajectory.q,'*');
% plot(trajectory.timeVector,trajectory.q_dot);
% plot(trajectory.timeVector,trajectory.q_ddot);

%% Test the ID effect
% % Create the config
% model_config = ModelConfig('PoCaBot spatial');
% % Load the SystemKinematics object from the XML
% modelObj = model_config.getModel('dualcables_large_endeffector_frame4by4by4_demo');
% 
% id_objective_Lin = IDObjectiveMinLinCableForce(ones(modelObj.numActuatorsActive,1));
% id_solver_Lin = IDSolverLinProg(modelObj, id_objective, ID_LP_SolverType.MATLAB);
% idsim_Lin = InverseDynamicsSimulator(modelObj, id_solver);
% 
% id_objective_Quad = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
% id_solver_Quad = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);
% idsim_Quad = InverseDynamicsSimulator(modelObj, id_solver);
% q0 = [1 1.985 1.120 0 0 0]';
% q1 = [0.788 1.985 1.120 0 0 0]';
% q2 = [0.788 1.985 1.020 0 0 0]';
% v_max = 0.120; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
% blend_time_default = 0.5; %used to decide the acceleration
% blend_time_placing = 2; %used to decide the deceleration
% trajectory1 = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q1, time_step, blend_time_default, blend_time_default, v_max);
% trajectory2 = PoCaBotExperiment.generateTrajectoryParabolicBlend(q1, q2, time_step, blend_time_default, blend_time_placing, v_max);
% fields = fieldnames(trajectory1)';
% fields(2,:) = cellfun(@(f) [trajectory1.(f) trajectory2.(f)], fields, 'unif', false);
% trajectory = struct(fields{:});
% force_Lin = [];
% force_Quad = [];
% for t = 1:1:length(trajectory.timeVector)
%     [~, model_temp, ~, ~, ~] = idsim_Lin.IDSolver.resolve(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t), zeros(idsim_Quad.model.numDofs,1));
%     force_Lin = [force_Lin model_temp.cableForces];
%     [~, model_temp, ~, ~, ~] = idsim_Quad.IDSolver.resolve(trajectory.q(:,t), trajectory.q_dot(:,t), trajectory.q_ddot(:,t), zeros(idsim_Quad.model.numDofs,1));
%     force_Quad = [force_Quad model_temp.cableForces];
% end
% plot(force_Lin');
% figure;
% plot(force_Quad');


