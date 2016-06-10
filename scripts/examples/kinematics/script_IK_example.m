% Script file to show how to use the inverse kinematics simulator
%
% Author        : Darwin LAU
% Created       : 2012
% Description    :

% Clear the variables, command window, and all windows
clear; clc; close all;

% Set up the type of model
model_config = ModelConfig(ModelConfigType.M_SIMPLE_PLANAR_XY);
cable_set_id = 'basic';
trajectory_id = 'x_simple';

% Load the SystemKinematics object from the XML
modelObj = model_config.getModel(cable_set_id);

% Setup the inverse kinematics simulator with the SystemKinematics object
disp('Start Setup Simulation');
sim = InverseKinematicsSimulator(modelObj);
trajectory = model_config.getTrajectory(trajectory_id);

% Run the kinematics on the desired trajectory
disp('Start Running Simulation');
sim.run(trajectory);

% After running the simulator the data can be plotted
% Refer to the simulator classes to see what can be plotted.
disp('Start Plotting Simulation');
sim.plotJointSpace([],[]);
sim.plotCableLengths([],[]);
sim.plotBodyCOG([],[]);
% sim.plotMovie(model_config.displayRange, [fileparts(mfilename('fullpath')), '\CDPR_movie.avi'], 2, 500, 640);