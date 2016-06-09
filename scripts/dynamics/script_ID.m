% Script file to show how to use the inverse dynamics simulator
%
% Author        : Darwin LAU
% Created       : 2012
% Description    :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
% Following are some examples (feel free to add more):
% 1) Planar model
model_config = ModelConfig(ModelConfigType.M_SIMPLE_PLANAR_XY);
cable_set_id = 'basic';
trajectory_id = 'x_simple';
% 2) Neck model
% model_config = ModelConfig(ModelConfigType.M_NECK_8S);
% cable_set_id = 'opensim_vasavada';
% trajectory_id = 'roll';
% 3) IPAnema model
% model_config = ModelConfig(ModelConfigType.M_IPANEMA_2);
% cable_set_id = 'original';
% trajectory_id = 'traj_z_up';

modelObj = model_config.getModel(cable_set_id);

id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numCables,1));
id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);

% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(modelObj, id_solver);
trajectory = model_config.getTrajectory(trajectory_id);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% Run the solver on the desired trajectory
disp('Start Running Simulation');
start_tic = tic;
idsim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

% Display information from the inverse dynamics simulator
fprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, total: %f seconds\n', mean(idsim.compTime), std(idsim.compTime), sum(idsim.compTime));

% Otherwise here is some simple example
disp('Start Plotting Simulation');
start_tic = tic;
% idsim.plotMovie(model_config.displayAxis, [fileparts(mfilename('fullpath')), '\CDPR_movie.avi'], 2, 500, 640);
idsim.plotJointSpace([],[]);
idsim.plotCableForces([],[]);
time_elapsed = toc(start_tic);
fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);