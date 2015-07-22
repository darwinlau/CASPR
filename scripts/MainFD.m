% Load configs
clc; clear; close all;

% Planar model
model_config = ModelConfig(ModelConfigType.M_PLANAR_XY);
trajectory_id = 'x_simple';
cable_set_id = 'basic';

% % 8S neck model
% model_config = ModelConfig(ModelConfigType.M_NECK_8S);
% trajectory_id = 'roll';
% cable_set_id = 'opensim_vasavada';

bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);

dynObj = SystemKinematicsDynamics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

idsolver = IDMinQuadCableForce(ones(dynObj.numCables,1));

disp('Start Setup Simulation');
start_tic = tic;

idsim = InverseDynamicsSimulator(dynObj, idsolver);
fdsim = ForwardDynamicsSimulator(dynObj);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, dynObj);

time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);


% Run inverse dynamics
disp('Start Running Inverse Dynamics Simulation');
start_tic = tic;

idsim.run(trajectory);

time_elapsed = toc(start_tic);
fprintf('End Running Inverse Dynamics Simulation : %f seconds\n', time_elapsed);

% Run forward dynamics
disp('Start Running Forward Dynamics Simulation');
start_tic = tic;

fdsim.run(idsim.cableForces, trajectory.timeVector, trajectory.q{1}, trajectory.q_dot{1});

time_elapsed = toc(start_tic);
fprintf('End Running Forward Dynamics Simulation : %f seconds\n', time_elapsed);