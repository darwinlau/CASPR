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

kinObj = SystemKinematics.LoadXmlObj(body_xmlobj, cable_xmlobj);
kinConstructor = @() SystemKinematics.LoadXmlObj(body_xmlobj, cable_xmlobj);

disp('Start Setup Simulation');
start_tic = tic;
sim = InverseKinematicsSimulator(kinObj);
trajectory = JointTrajectory.LoadXmlObj(trajectories_xmlobj.getElementById(trajectory_id), kinObj);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

disp('Start Running Simulation');
start_tic = tic;
sim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

% disp('Start Plotting Simulation');
% start_tic = tic;
% %plot_axis = [0 1 0 1 -0.1 0.1];
% plot_axis = [-0.2 0.2 -0.2 0.2 -0.1 0.3];
% sim.plotMovie(sim.trajectory, plot_axis, [folder, '\test.avi'], 2, 500, 640);
% sim.plotJointSpace(sim.trajectory);
% sim.plotAngularAcceleration(sim.trajectory);
% sim.plotCableLengths(sim.lengths, sim.lengths_dot);
% sim.plotBodyCOG(sim.trajectory);
% time_elapsed = toc(start_tic);
% fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);

