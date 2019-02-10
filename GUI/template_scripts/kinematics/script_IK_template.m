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
CASPR_log.Info('Start Setup Simulation');
sim = InverseKinematicsSimulator(modelObj);
trajectory = model_config.getJointTrajectory(trajectory_id);
CASPR_log.Info('Finished Setup Simulation');

% Run the kinematics on the desired trajectory
CASPR_log.Info('Start Running Simulation');
sim.run(trajectory);
CASPR_log.Info('Finished Running Simulation');

% After running the simulator the data can be plotted
% Refer to the simulator classes to see what can be plotted.
CASPR_log.Info('Start Plotting Simulation');
sim.plotJointSpace();
sim.plotCableLengths();
CASPR_log.Info('Finished Plotting Simulation');
