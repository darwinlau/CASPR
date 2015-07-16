% Load configs
clc; clear; close all;
folder = 'D:\Darwin''s Notebook Documents\Work\Research\Studies\Cable-driven manipulators\Simulations\MCDM_matlab\data';

% % Planar model
% body_prop_filepath = [folder, '\config\models\planar\planar_body_properties.xml'];
% cable_prop_filepath = [folder, '\config\models\planar\planar_cable_properties.xml'];
% trajectories_filepath = [folder, '\config\models\planar\planar_trajectories.xml'];
% trajectory_id = 'x_simple';

% 8S model
body_prop_filepath = [folder, '\config\models\8S_neck\8S_neck_body_properties.xml'];
cable_prop_filepath = [folder, '\config\models\8S_neck\8S_neck_cable_properties.xml'];
trajectories_filepath = [folder, '\config\models\8S_neck\8S_neck_trajectories.xml'];
trajectory_id = 'roll';

body_xmlobj = XmlOperations.XmlReadRemoveIndents(body_prop_filepath);
cable_xmlobj = XmlOperations.XmlReadRemoveIndents(cable_prop_filepath);
trajectories_xmlobj = XmlOperations.XmlReadRemoveIndents(trajectories_filepath);

dynObj = SystemKinematicsDynamics.LoadXmlObj(body_xmlobj, cable_xmlobj);

idsolver = IDMinQuadCableForce(ones(dynObj.numCables,1));

disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(dynObj, idsolver);
fdsim = ForwardDynamicsSimulator(dynObj);
trajectory = JointTrajectory.LoadXmlObj(trajectories_xmlobj.getElementById(trajectory_id), dynObj);
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