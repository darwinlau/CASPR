% Script for inverse dynamics (ID) using feasible polygon ID solver
% NOTE: Number of cables must be exactly 2 more than the system DoFs
%
% Author        : Autogenerate
% Created       : 20XX
% Description   :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
model_config = ModelConfig('Example planar XY');
cable_set_id = 'basic';
trajectory_id = 'example_quintic';

modelObj = model_config.getModel(cable_set_id);
id_solver = IDSolverClosedForm(modelObj, ID_CF_SolverType.IMPROVED_CLOSED_FORM);

% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
idsim = InverseDynamicsSimulator(modelObj, id_solver);
trajectory = model_config.getJointTrajectory(trajectory_id);
disp('Finished Setup Simulation');

% Run the solver on the desired trajectory
disp('Start Running Simulation');
idsim.run(trajectory);
disp('Finished Running Simulation');

% Display information from the inverse dynamics simulator
disp(sprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, total: %f seconds', mean(idsim.compTime), std(idsim.compTime), sum(idsim.compTime)));

% Plotting simulation graphs
disp('Start Plotting Simulation');
idsim.plotJointSpace();
idsim.plotCableForces();
disp('Finished Plotting Simulation');