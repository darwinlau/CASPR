% Script file to show how to use the inverse kinematics simulator
%
% Author        : Darwin LAU
% Created       : 2012
% Description    :

% Clear the variables, command window, and all windows
clear; clc; close all;

% Set up the type of model, trajectory and the set of cables to be used
% Following are some examples (feel free to add more):
% 1) Planar model
% model_config = ModelConfig(ModelConfigType.M_SIMPLE_PLANAR_XY);
% trajectory_id = 'x_simple';
% cable_set_id = 'basic';
% 2) Neck model
model_config = ModelConfig(ModelConfigType.M_NECK_8S);
trajectory_id = 'pitch';
cable_set_id = 'opensim_vasavada';

% The XML objects from the model config are created
bodies_xmlobj = model_config.getBodiesPropertiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);

% Load the SystemKinematics object from the XML
kinObj = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

% Setup the inverse kinematics simulator with the SystemKinematics object
disp('Start Setup Simulation');
start_tic = tic;
sim = InverseKinematicsSimulator(kinObj);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, kinObj);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% Run the kinematics on the desired trajectory
disp('Start Running Simulation');
start_tic = tic;
sim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

% After running the simulator the data can be plotted
% Refer to the simulator classes to see what can be plotted.
disp('Start Plotting Simulation');
start_tic = tic;
plot_axis = [0 1 0 1 -0.1 0.1];
% plot_axis = [-0.2 0.2 -0.2 0.2 -0.1 0.3];
% sim.plotMovie(plot_axis, [fileparts(mfilename('fullpath')), '\test.avi'], 2, 500, 640);
sim.plotJointSpace([],[]);
% sim.plotAngularAcceleration();
% sim.plotCableLengths();
sim.plotBodyCOG([],[]);
time_elapsed = toc(start_tic);
fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
