% Script for forward kinematics (FK) using differential method
%
% Author        : Autogenerate
% Created       : 20XX
% Description   :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model
model_config = ModelConfig('Example planar XY');
cable_set_id = 'basic';
trajectory_id = 'example_linear';

% Load the SystemKinematics object from the XML
modelObj = model_config.getModel(cable_set_id);

% An inverse kinematics and forward kinematics simulator will be run to
% show that the results are consistent.
disp('Start Setup Simulation');
% Initialise the least squares solver for the forward kinematics
fksolver = FKDifferential(modelObj);
% Initialise the inverse/forward kinematics solvers
iksim = InverseKinematicsSimulator(modelObj);
fksim = ForwardKinematicsSimulator(modelObj, fksolver);
trajectory = model_config.getJointTrajectory(trajectory_id);
disp('Finished Setup Simulation');

% Run inverse kinematics
disp('Start Running Inverse Kinematics Simulation');
iksim.run(trajectory);
disp('Finished Running Inverse Kinematics Simulation');

% Run forward kinematics
disp('Start Running Forward Kinematics Simulation');
fksim.run(iksim.cableLengths, iksim.cableLengthsDot, iksim.timeVector, iksim.trajectory.q{1}, iksim.trajectory.q_dot{1});
disp('Finished Running Forward Kinematics Simulation');

% It is expected that iksim and fksim should have the same joint space (the
% result of fksim)
disp('Start Plotting Simulation');
iksim.plotJointSpace();
fksim.plotJointSpace();
fksim.plotCableLengthError();
disp('Finished Plotting Simulation');