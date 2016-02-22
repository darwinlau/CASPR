% Script file to show how to use the foward kinematics simulator
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
% Following are some examples (feel free to add more):
% 1) Planar model
% model_config = ModelConfig(ModelConfigType.M_SIMPLE_PLANAR_XY);
% trajectory_id = 'general_1';
% cable_set_id = 'basic';
% 2) Neck model
% model_config = ModelConfig(ModelConfigType.M_NECK_8S);
% trajectory_id = 'roll';
% cable_set_id = 'opensim_vasavada';
% 3) TUM Myorob arm model
% model_config = ModelConfig(ModelConfigType.M_MYOROB_SHOULDER);
% trajectory_id = 'traj_1';
% cable_set_id = 'default';
% 4) IPAnema model
model_config = ModelConfig(ModelConfigType.M_IPANEMA_2);
trajectory_id = 'traj_1';
cable_set_id = 'default';

% The XML objects from the model config are created
bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);

% Load the SystemKinematics object from the XML
kinObj = SystemKinematics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

% An inverse kinematics and forward kinematics simulator will be run to
% show that the results are consistent.
disp('Start Setup Simulation');
start_tic = tic;
% Initialise the least squares solver for the forward kinematics
fksolver = FKLeastSquares(kinObj, FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_PSEUDOINV, FK_LS_QdotOptionType.FIRST_ORDER_DERIV);
%fksolver = FKDifferential();
%fksolver = FKDifferential2();
% Initialise the three inverse/forward kinematics solvers
iksim = InverseKinematicsSimulator(kinObj);
fksim = ForwardKinematicsSimulator(kinObj, fksolver);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, kinObj);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% Run inverse kinematics
disp('Start Running Inverse Kinematics Simulation');
start_tic = tic;
iksim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Inverse Kinematics Simulation : %f seconds\n', time_elapsed);

% Run forward kinematics
disp('Start Running Forward Kinematics Simulation');
start_tic = tic;
fksim.run(iksim.cableLengths, iksim.cableLengthsDot, iksim.timeVector, iksim.trajectory.q{1}, iksim.trajectory.q_dot{1});
time_elapsed = toc(start_tic);
fprintf('End Running Forward Kinematics Simulation : %f seconds\n', time_elapsed);

% It is expected that iksim and fksim should have the same joint space (the
% result of fksim)
iksim.plotJointSpace();
fksim.plotJointSpace();
fksim.plotCableLengthError();
