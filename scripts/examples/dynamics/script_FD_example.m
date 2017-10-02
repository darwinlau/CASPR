% Script file to show how to use the forward dynamics simulator
% 
% Author        : Autogenerate
% Created       : 20XX
% Description	:

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
model_config = ModelConfig('Example planar XY');
cable_set_id = 'basic';
trajectory_id = 'example_quintic';

modelObj = model_config.getModel(cable_set_id);

% Setup an inverse dynamics solver of choice (any should do)
id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);

% Setup the inverse dynamics and forward dynamics simulators
disp('Start Setup Simulation');
start_tic = tic;
% Create the inverse dynamics simulator
idsim = InverseDynamicsSimulator(modelObj, id_solver);
% Create the forward dynamics simulator
% NOTE: ForwardDynamicsSimulator has changable input fd_solver_type.
%       This can be set to any of the enums in FDSolverType.
fdsim = ForwardDynamicsSimulator(modelObj, FDSolverType.ODE113);
trajectory = model_config.getJointTrajectory(trajectory_id);
time_elapsed = toc(start_tic);
disp(sprintf('End Setup Simulation : %f seconds', time_elapsed));

% First run the inverse dynamics
disp('Start Running Inverse Dynamics Simulation');
start_tic = tic;
idsim.run(trajectory);
time_elapsed = toc(start_tic);
disp(sprintf('End Running Inverse Dynamics Simulation : %f seconds',time_elapsed));

% Then run the forward dynamics
disp('Start Running Forward Dynamics Simulation');
start_tic = tic;
fdsim.run(idsim.cableForcesActive, idsim.cableIndicesActive, trajectory.timeVector, trajectory.q{1}, trajectory.q_dot{1});
time_elapsed = toc(start_tic);
disp(sprintf('End Running Forward Dynamics Simulation : %f seconds', time_elapsed));

% Finally compare the results
idsim.plotJointSpace();
fdsim.plotJointSpace();
