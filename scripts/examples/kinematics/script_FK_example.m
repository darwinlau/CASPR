% Script file to show how to use the foward kinematics simulator
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model
model_config = ModelConfig('Example planar XY');
cable_set_id = 'basic';
trajectory_id = 'x_simple';

% Load the SystemKinematics object from the XML
modelObj = model_config.getModel(cable_set_id);

% An inverse kinematics and forward kinematics simulator will be run to
% show that the results are consistent.
disp('Start Setup Simulation');
% Initialise the least squares solver for the forward kinematics
fksolver = FKLeastSquares(modelObj, FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_PSEUDOINV, FK_LS_QdotOptionType.FIRST_ORDER_DERIV);
% Initialise the inverse/forward kinematics solvers
iksim = InverseKinematicsSimulator(modelObj);
fksim = ForwardKinematicsSimulator(modelObj, fksolver);
trajectory = model_config.getJointTrajectory(trajectory_id);

% Run inverse kinematics
disp('Start Running Inverse Kinematics Simulation');
iksim.run(trajectory);

% Run forward kinematics
disp('Start Running Forward Kinematics Simulation');
fksim.run(iksim.cableLengths, iksim.cableLengthsDot, iksim.timeVector, iksim.trajectory.q{1}, iksim.trajectory.q_dot{1});

% It is expected that iksim and fksim should have the same joint space (the
% result of fksim)
iksim.plotJointSpace();
fksim.plotJointSpace();
fksim.plotCableLengthError();
