% Script file to show how to use the inverse dynamics simulator
%
% Author        : Chen SONG
% Created       : 2016
% Description   : 
%     An example for Varying Gain Computed Torque Control

% Clear the variables, command window, and all windows
clc; clear; close all;
warning off;
% Set up the type of model, trajectory and the set of cables:
model_config = ModelConfig(ModelConfigType.M_IPANEMA_2);
trajectory_id = 'traj_z_up';
cable_set_id = 'original';

% Load the SystemModel object
modelObj = model_config.getModel(cable_set_id);

% Set the minimum control gains
kp_min = 50*ones(modelObj.numDofs, 1);
kd_min = 15*ones(modelObj.numDofs, 1);

controller = VaryingGainCTCLsqnonneg(modelObj, kp_min, kd_min);

% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
fdSolver = ForwardDynamics(FDSolverType.ODE113);
control_sim = ControllerSimulator(modelObj, controller, fdSolver);
trajectory_ref = model_config.getTrajectory(trajectory_id);

% Run the solver on the desired trajectory
disp('Start Running Simulation');
error0 = [0.1; -0.1; 0.2; 0; 0; 0];
control_sim.run(trajectory_ref, trajectory_ref.q{1} + error0, trajectory_ref.q_dot{1}, trajectory_ref.q_ddot{1});

control_sim.plotCableForces();
control_sim.plotJointSpaceTracking();
control_sim.plotTrackingError();
