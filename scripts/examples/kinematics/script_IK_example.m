% Script for inverse kinematics (IK) 
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

% Setup the inverse kinematics simulator with the SystemKinematics object
disp('Start Setup Simulation');
sim = InverseKinematicsSimulator(modelObj);
trajectory = model_config.getJointTrajectory(trajectory_id);
disp('Finished Setup Simulation');

% Run the kinematics on the desired trajectory
disp('Start Running Simulation');
sim.run(trajectory);
disp('Finished Running Simulation');

% After running the simulator the data can be plotted
% Refer to the simulator classes to see what can be plotted.
disp('Start Plotting Simulation');
sim.plotJointSpace();
sim.plotCableLengths();
disp('Finished Plotting Simulation');
