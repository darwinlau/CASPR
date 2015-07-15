% Load configs
clc; clear; close all;
folder = 'D:\Darwin''s Notebook Documents\Work\Research\Studies\Cable-driven manipulators\Simulations\MCDM_matlab\data\config';

% % Planar model
% body_prop_filepath = [folder, '\models\planar\planar_body_properties.xml'];
% cable_prop_filepath = [folder, '\models\planar\planar_cable_properties.xml'];
% trajectories_filepath = [folder, '\models\planar\planar_trajectories.xml'];
% trajectory_id = 'x_simple';

% 8S model
body_prop_filepath = [folder, '\models\8S_neck\8S_neck_body_properties.xml'];
cable_prop_filepath = [folder, '\models\8S_neck\8S_neck_cable_properties.xml'];
trajectories_filepath = [folder, '\models\8S_neck\8S_neck_trajectories.xml'];
trajectory_id = 'roll';




%trajec_prop_filepath = [folder, '\systems_prop\SR_model\SR_trajectory.csv'];


% Neck Model
% cables_prop_filepath = [folder, '\systems_prop\8S_neck_model\8S_neck_ideal_cable_prop.csv'];
% bodies_prop_filepath = [folder, '\systems_prop\8S_neck_model\8S_neck_body_prop.csv'];
%trajec_prop_filepath = [folder, '\systems_prop\8S_neck_model\8S_neck_trajectory_roll.csv'];
%trajec_prop_filepath = [folder, '\systems_prop\8S_neck_model\8S_neck_trajectory_gen.csv'];

% cables_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_cables_prop.csv'];
% bodies_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_body_prop.csv'];
% trajec_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_trajectory.csv'];

% cables_prop_filepath = [folder, '\systems_prop\planar_model\planar_ideal_cable_props.csv'];
% bodies_prop_filepath = [folder, '\systems_prop\planar_model\planar_rigid_body_props.csv'];
% trajec_prop_filepath = [folder, '\systems_prop\planar_model\planar_trajectory.csv'];

body_xmlobj = XmlOperations.XmlReadRemoveIndents(body_prop_filepath);
cable_xmlobj = XmlOperations.XmlReadRemoveIndents(cable_prop_filepath);
trajectories_xmlobj = XmlOperations.XmlReadRemoveIndents(trajectories_filepath);

kinConstructor = @() SystemKinematics.LoadXmlObj(body_xmlobj, cable_xmlobj);

disp('Start Setup Simulation');
start_tic = tic;
sim = InverseKinematicsSimulator();
trajectory = JointTrajectory.LoadXmlObj(trajectories_xmlobj.getElementById(trajectory_id), kinConstructor());
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

disp('Start Running Simulation');
start_tic = tic;
sim.run(trajectory, kinConstructor);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

% disp('Start Plotting Simulation');
% start_tic = tic;
% %plot_axis = [0 1 0 1 -0.1 0.1];
% plot_axis = [-0.2 0.2 -0.2 0.2 -0.1 0.3];
% sim.plotMovie(plot_axis, [folder, '\test.avi'], 2, 500, 640);
% time_elapsed = toc(start_tic);
% sim.plotJointSpace();
% sim.plotAngularAcceleration();
% sim.plotCableLengths();
% sim.plotBodyCOG();
% fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);

