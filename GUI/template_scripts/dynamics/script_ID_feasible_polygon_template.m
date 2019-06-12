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
trajectory_id = 'example_linear';

modelObj = model_config.getModel(cable_set_id);
id_solver = IDSolverFeasiblePolygon(modelObj, ID_FP_SolverType.NORM_2);

% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
CASPR_log.Info('Start Setup Simulation');
idsim = InverseDynamicsSimulator(modelObj, id_solver);
trajectory = model_config.getJointTrajectory(trajectory_id);
CASPR_log.Info('Finished Setup Simulation');

% Run the solver on the desired trajectory
CASPR_log.Info('Start Running Simulation');
idsim.run(trajectory);
CASPR_log.Info('Finished Running Simulation');

% Display information from the inverse dynamics simulator
CASPR_log.Info(sprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, total: %f seconds', mean(idsim.compTime), std(idsim.compTime), sum(idsim.compTime)));

% Plotting simulation graphs
CASPR_log.Info('Start Plotting Simulation');
idsim.plotJointSpace();
idsim.plotCableForces();
CASPR_log.Info('Finished Plotting Simulation');