% Script file to show how to use the inverse dynamics simulator
% 
% Author        : Darwin LAU
% Created       : 2012
% Description	:
% Known issues  : 1) Possible issues with the ODE solvers for complex
%                    problems

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
model_config = ModelConfig('Example planar XY');
cable_set_id = 'basic';
trajectory_id = 'x_simple';

modelObj = model_config.getModel(cable_set_id);

% Setup an inverse dynamics solver of choice (any should do)
id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);

% Setup the inverse dynamics and forward dynamics simulators
disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(modelObj, id_solver);
fdsim = ForwardDynamicsSimulator(modelObj, FDSolverType.ODE113);
trajectory = model_config.getJointTrajectory(trajectory_id);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% First run the inverse dynamics
disp('Start Running Inverse Dynamics Simulation');
start_tic = tic;
idsim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Inverse Dynamics Simulation : %f seconds\n', time_elapsed);

% Then run the forward dynamics
disp('Start Running Forward Dynamics Simulation');
start_tic = tic;
fdsim.run(idsim.cableForcesActive, idsim.cableIndicesActive, trajectory.timeVector, trajectory.q{1}, trajectory.q_dot{1});
time_elapsed = toc(start_tic);
fprintf('End Running Forward Dynamics Simulation : %f seconds\n', time_elapsed);

% Finally compare the results
idsim.plotJointSpace();
fdsim.plotJointSpace();
