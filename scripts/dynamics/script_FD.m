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
% Following are some examples (feel free to add more):
% 1) Planar model
% model_config = ModelConfig(ModelConfigType.M_SIMPLE_PLANAR_XY);
% trajectory_id = 'x_simple';
% cable_set_id = 'basic';
% 2) Neck model
% model_config = ModelConfig(ModelConfigType.M_NECK_8S);
% trajectory_id = 'roll';
% cable_set_id = 'opensim_vasavada';
% 3) TUM Myorob arm model
model_config = ModelConfig(ModelConfigType.M_MYOROB_SHOULDER);
trajectory_id = 'traj_1';
cable_set_id = 'default';
% 4) IPAnema model
% model_config = ModelConfig(ModelConfigType.M_IPANEMA_2);
% trajectory_id = 'traj_z_up';
% cable_set_id = 'default';

% The XML objects from the model config are created
bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);

% Load the SystemKinematicsDynamics object from the XML
dynObj = SystemKinematicsDynamics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

% Setup an inverse dynamics solver of choice (any should do)
id_objective = IDObjectiveMinQuadCableForce(ones(dynObj.numCables,1));
id_solver = IDSolverQuadProg(dynObj, id_objective, ID_QP_SolverType.OPTITOOLBOX_OOQP);

% Setup the inverse dynamics and forward dynamics simulators
disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(dynObj, id_solver);
fdsim = ForwardDynamicsSimulator(dynObj);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, dynObj);
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
fdsim.run(idsim.cableForces, trajectory.timeVector, trajectory.q{1}, trajectory.q_dot{1});
time_elapsed = toc(start_tic);
fprintf('End Running Forward Dynamics Simulation : %f seconds\n', time_elapsed);

% Finally compare the results
idsim.plotJointSpace([], []);
fdsim.plotJointSpace([], []);
% 
% plot_axis = [-5 5 -5 5 0 5];
% idsim.plotMovie(plot_axis, [fileparts(mfilename('fullpath')), '\test.avi'], 2, 500, 640);
