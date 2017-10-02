% Script file for control using CTC
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables:
model_config = ModelConfig('Example planar XY');
cable_set_id = 'basic';
trajectory_id = 'example_quintic';

% Load the SystemModel object
modelObj = model_config.getModel(cable_set_id);
id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numCables,1));
id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);
Kp_computedtorque = 50*eye(modelObj.numDofs);
Kd_computedtorque = 15*eye(modelObj.numDofs);
controller = LyapunovStaticCompensation(modelObj, id_solver, Kp_computedtorque, Kd_computedtorque);

% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
fdSolver = ForwardDynamics(FDSolverType.ODE113);
control_sim = ControllerSimulator(modelObj, controller, fdSolver);
trajectory_ref = model_config.getJointTrajectory(trajectory_id);

% Run the solver on the desired trajectory
disp('Start Running Simulation');
n_q = modelObj.numDofs;
initial_pose_error = 0.2*rand(n_q,1) - 0.1*ones(n_q,1);
control_sim.run(trajectory_ref, trajectory_ref.q{1} + initial_pose_error, trajectory_ref.q_dot{1}, trajectory_ref.q_ddot{1});
disp('Finished Running Simulation');

% Plotting simulation graphs
disp('Start Plotting Simulation');
control_sim.plotCableForces();
control_sim.plotJointSpaceTracking();
control_sim.plotTrackingError();
disp('Finished Plotting Simulation');
