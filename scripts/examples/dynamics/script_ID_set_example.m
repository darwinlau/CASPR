% Script for inverse dynamics (ID) using feasible polygon ID solver
% NOTE: Number of cables must be exactly 2 more than the system DoFs
%
% Author        : Autogenerate
% Created       : 20XX
% Description   :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
model_config = ModelConfig('IPAnema 2');
cable_set_id = 'original';
trajectory_id = 'traj_z_up';

modelObj = model_config.getModel(cable_set_id);
id_objective_q = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
id_objective_l = IDObjectiveMinLinCableForce(ones(modelObj.numActuatorsActive,1));
id_objective_i = IDObjectiveMinInfCableForce(ones(modelObj.numActuatorsActive,1));
id_solvers = {IDSolverQuadProg(modelObj, id_objective_q, ID_QP_SolverType.MATLAB),...
              IDSolverOptimallySafe(modelObj, 1.0, ID_OS_SolverType.EFFICIENT_LP),...
              IDSolverClosedForm(modelObj, ID_CF_SolverType.IMPROVED_CLOSED_FORM),...
              IDSolverLinProg(modelObj, id_objective_l, ID_LP_SolverType.MATLAB),...
              IDSolverMinInfNorm(modelObj, id_objective_i, ID_LP_SolverType.MATLAB)};

% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
idsim = InverseDynamicsSimulatorSet(modelObj, id_solvers);
trajectory = model_config.getJointTrajectory(trajectory_id);
disp('Finished Setup Simulation');

% Run the solver on the desired trajectory
disp('Start Running Simulation');
idsim.run(trajectory);
disp('Finished Running Simulation');

% Display information from the inverse dynamics simulator
%disp(sprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, total: %f seconds', mean(idsim.compTime), std(idsim.compTime), sum(idsim.compTime)));

% Plotting simulation graphs
disp('Start Plotting Simulation');
idsim.plotCompCost;
disp('Finished Plotting Simulation');
