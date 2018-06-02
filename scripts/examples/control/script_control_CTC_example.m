% Script file for control using CTC
%
% Author        : Chen SONG
% Created       : 2017
% Description   : 
%     Script file for control using CTC. Currently GUI doesn't work with
%     the new controller simulator, this will be a template for the
%     simulator use.

% Clear the variables, command window, and all windows, set environment
clc; clear; close all;

disp('Start Setup Simulation');
% Set up the type of model, trajectory and the set of cables:
model_config = ModelConfig('BMArm');
trajectory_id = 'traj_test';
cable_set_id = 'WORKING';

% Construct SystemModel objects
modelObj     =   model_config.getModel(cable_set_id);

% Construct Reference Trajectory objects
trajectory_ref = model_config.getJointTrajectory(trajectory_id);

% Construct FD object
fd_solver = ForwardDynamics(FDSolverType.ODE4);
id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);

% Construct Controller objects
damping_ratio = 1;
target_ratio = 0.6;
kp_ub = 1000;
kd_ub = 2*sqrt(kp_ub)*damping_ratio;
kp_tar = kp_ub*target_ratio;
kd_tar = 2*sqrt(kp_tar)*damping_ratio;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% constant CTC
controller = ComputedTorqueController(modelObj, id_solver, kp_tar*eye(modelObj.numDofs), kd_tar*eye(modelObj.numDofs));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ctrl_sim = ControllerSimulator(modelObj, controller, fd_solver, [], [], modelObj, [], []);

disp('Finished Setup Simulation');

% Run the solver on the desired trajectory
disp('Start Running Simulation');
ctrl_sim.run(trajectory_ref, trajectory_ref.q{1}, trajectory_ref.q_dot{1}, zeros(modelObj.numDofs, 1));
