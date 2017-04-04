% Script file to show how to simply load a robot
%
% Author        : Jonathan EDEN
% Created       : 2016
% Description    :

% Clear the variables, command window, and all windows
clear; clc; close all;

% Set up the type of model, trajectory and the set of cables to be used
% Planar model
model_config = ModelConfig('Example planar XY');
modelObj = model_config.getModel('basic');

% Create an inverse dynamics solver
id_type = ID_CF_SolverType.IMPROVED_CLOSED_FORM;
id_solver = IDSolverClosedForm(modelObj, id_type);

% Setup the inverse dynamics simulator with the SystemModel
% object and the inverse dynamics solver
idsim = InverseDynamicsSimulator(modelObj, id_solver);
trajectory = model_config.getTrajectory('x_simple');

% Run the solver on the desired trajectory
idsim.run(trajectory);

% Plotting simulation graphs
idsim.plotJointSpace();
idsim.plotCableForces();
