% Script file for control using CTC
%
% Author        : Chen SONG
% Created       : 2017
% Description   : 
%     Script file for control using CTC. Currently GUI doesn't work with
%     the new controller simulator, this will be a template for the
%     simulator use.

% Clear the variables, command window, and all windows, set environment
clc; clear; close all;
warning off;

disp('Start Setup Simulation');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% DO YOUR DEFINITION HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the type of model, trajectory and the set of cables:
model_config = ModelConfig('BMArm');
trajectory_id = 'traj_test';
cable_set_id = 'WORKING';

% Construct SystemModel objects
ideal_model     =   model_config.getModel(cable_set_id);
true_model      =   model_config.getModel(cable_set_id);

% Construct Reference Trajectory objects
trajectory_ref = model_config.getJointTrajectory(trajectory_id);

% Parameters for controller simulator
sim_freq_ratio	=   1;
ob_freq_ratio	=   1;
ref_delta_t     =   trajectory_ref.timeVector(2) - trajectory_ref.timeVector(1);
sim_delta_t     =   ref_delta_t/sim_freq_ratio;
ob_delta_t      =   ref_delta_t/ob_freq_ratio;

% Construct controller simulator options
simopt = ControllerSimulatorOptions();
% simulation frequency
simopt = ControllerSimulatorOptions(simopt, 'SimulationFrequencyRatio', sim_freq_ratio);
% observer frequency
simopt = ControllerSimulatorOptions(simopt, 'ObserverFrequencyRatio', ob_freq_ratio);
% absolute encoder
simopt = ControllerSimulatorOptions(simopt, 'UseAbsoluteEncoder', 'true');
% FK usage - enable FK
simopt = ControllerSimulatorOptions(simopt, 'EnableFKSolver', 'false');
% FK usage - use FK in controller
simopt = ControllerSimulatorOptions(simopt, 'UseFKInController', 'false');
% FK usage - use FK in observer
simopt = ControllerSimulatorOptions(simopt, 'UseFKInObserver', 'false');
% FK usage - debug FK solvers
simopt = ControllerSimulatorOptions(simopt, 'FKDebugging', 'false');
% observer usage - enable observer
simopt = ControllerSimulatorOptions(simopt, 'EnableObserver', 'false');
% observer usage - use estimated disturbance
simopt = ControllerSimulatorOptions(simopt, 'UseDisturbanceEstimation', 'false');
% observer usage - use estimated state
simopt = ControllerSimulatorOptions(simopt, 'UseStateEstimation', 'false');

% Construct FD object
% fd_solver = ForwardDynamics(FDSolverType.ODE113);
fd_solver = ForwardDynamics(FDSolverType.ODE4);
% fd_solver = ForwardDynamics(FDSolverType.ODE45);

% Construct ID object
id_objective = IDObjectiveMinQuadCableForce(ones(ideal_model.numCables,1));
id_solver = IDSolverQuadProg(ideal_model, id_objective, ID_QP_SolverType.MATLAB);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% DO YOUR DEFINITION HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Construct Controller objects
damping_ratio = 1;
target_ratio = 0.6;
kp_ub = 1000;
kd_ub = 2*sqrt(kp_ub)*damping_ratio;
kp_tar = kp_ub*target_ratio;
kd_tar = 2*sqrt(kp_tar)*damping_ratio;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% constant CTC
controller = ComputedTorqueController(ideal_model, id_solver, kp_tar*eye(ideal_model.numDofs), kd_tar*eye(ideal_model.numDofs));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% Construct Disturbance objects
num_of_uncertainties    =   0;
uncertainties           =   [];
% 3) Disturbance uncertainties
num_of_uncertainties    =   num_of_uncertainties + 1;
time_sequence           =   [1; 3];
disturbance_sequence    =   [2, 0, 0, 0; 0, 0, 0, 0];
uncertainties{num_of_uncertainties} = ExternalWrenchUncertaintyDisccreteSequence(true_model, time_sequence, disturbance_sequence);


ctrl_sim = ControllerSimulator(ideal_model, controller, fd_solver, [], uncertainties, true_model, [], simopt);

disp('Finished Setup Simulation');




% Run the solver on the desired trajectory
disp('Start Running Simulation');
error0 = [0.0; 0.0; 0.0; 0.0];
tic
ctrl_sim.run(trajectory_ref, trajectory_ref.q{1} + error0, trajectory_ref.q_dot{1}, zeros(ideal_model.numDofs, 1));
toc
output_data     =   ctrl_sim.extractData();
repo_folder = pwd;
data_folder = '';
complete_folder = strcat(repo_folder, data_folder);


len = min([size(output_data.DataRefTime, 1), size(output_data.DataRefPose, 1), size(output_data.DataCtrlJointPose, 1)]);
DataRefTime_ctrl            =   output_data.DataRefTime(1:len, :);
DataCtrlJointPose_ctrl  	=   output_data.DataCtrlJointPose(1:len, :);
DataRefPose_ctrl   	=   output_data.DataRefPose(1:len, :);


len = min([size(output_data.DataRefTime, 1), size(output_data.DataRefPose, 1), size(output_data.DataSimJointPose, 1)]);
DataRefTime_sim         =   output_data.DataRefTime(1:len, :);
DataSimJointPose_sim   	=   output_data.DataSimJointPose(1:len, :);
DataRefPose_sim   	=   output_data.DataRefPose(1:len, :);
figure;
plot(DataRefTime_ctrl, DataCtrlJointPose_ctrl, 'LineWidth', 1.0);
hold on;
plot(DataRefTime_ctrl, DataRefPose_ctrl, 'LineWidth', 1.0);
title('Joint Tracking');
legend('FK Joint 1','FK Joint 2','FK Joint 3','FK Joint 4', 'Ref Joint 1','Ref Joint 2','Ref Joint 3','Ref Joint 4')
% saveas(gcf, strcat(complete_folder, '\compare_q_traj.eps'));
% saveas(gcf, strcat(complete_folder, '\compare_q_traj.png'));

figure;
plot(DataRefTime_sim, DataSimJointPose_sim, 'LineWidth', 1.0);
hold on;
plot(DataRefTime_sim, DataRefPose_sim, 'LineWidth', 1.0);
title('Joint Tracking (Actual)');
legend('Env Joint 1','Env Joint 2','Env Joint 3','Env Joint 4', 'Ref Joint 1','Ref Joint 2','Ref Joint 3','Ref Joint 4')
% saveas(gcf, strcat(complete_folder, '\compare_q_traj_real.eps'));
% saveas(gcf, strcat(complete_folder, '\compare_q_traj_real.png'));

figure;
plot(DataRefTime_ctrl, DataCtrlJointPose_ctrl - DataRefPose_ctrl, 'LineWidth', 1.5);
title('Joint Tracking Error');
legend('Joint 1','Joint 2','Joint 3','Joint 4')
% saveas(gcf, strcat(complete_folder, '\err_q_traj.eps'));
% saveas(gcf, strcat(complete_folder, '\err_q_traj.png'));

figure;
plot(DataRefTime_sim, DataSimJointPose_sim - DataRefPose_sim, 'LineWidth', 1.5);
title('Joint Tracking Error (Actual)');
legend('Joint 1','Joint 2','Joint 3','Joint 4')
% saveas(gcf, strcat(complete_folder, '\err_q_traj_real.eps'));
% saveas(gcf, strcat(complete_folder, '\err_q_traj_real.png'));

figure;
plot(output_data.DataCtrlTime, output_data.DataCtrlForceCommands, 'LineWidth', 1.5);
title('Force Commands');
legend('Cable 1','Cable 2','Cable 3','Cable 4','Cable 5','Cable 6')
% saveas(gcf, strcat(complete_folder, '\f_cmd.eps'));
% saveas(gcf, strcat(complete_folder, '\f_cmd.png'));