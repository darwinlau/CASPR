% Script file to show how to use the inverse dynamics simulator
%
% Author        : Darwin LAU
% Created       : 2012
% Description    :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
model_config = ModelConfig('Example planar XY');
cable_set_id = 'basic';
trajectory_id = 'x_simple';

modelObj = model_config.getModel(cable_set_id);

id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);

% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
idsim = InverseDynamicsSimulator(modelObj, id_solver);
trajectory = model_config.getJointTrajectory(trajectory_id);

% Run the solver on the desired trajectory
disp('Start Running Simulation');
idsim.run(trajectory);

% Display information from the inverse dynamics simulator
fprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, total: %f seconds\n', mean(idsim.compTime), std(idsim.compTime), sum(idsim.compTime));

% Plotting simulation graphs
disp('Start Plotting Simulation');
idsim.plotJointSpace();
idsim.plotCableForces();
% idsim.plotMovie(model_config.displayAxis, [fileparts(mfilename('fullpath')), '\CDPR_movie.avi'], 2, 500, 640);
