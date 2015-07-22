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

kinObj = SystemKinematics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

disp('Start Setup Simulation');
start_tic = tic;
sim = InverseKinematicsSimulator(kinObj);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, kinObj);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

disp('Start Running Simulation');
start_tic = tic;
sim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

disp('Start Plotting Simulation');
start_tic = tic;
plot_axis = [0 1 0 1 -0.1 0.1];
% plot_axis = [-0.2 0.2 -0.2 0.2 -0.1 0.3];
sim.plotMovie(plot_axis, [folder, '\test.avi'], 2, 500, 640);
sim.plotJointSpace();
sim.plotAngularAcceleration();
sim.plotCableLengths();
sim.plotBodyCOG();
time_elapsed = toc(start_tic);
fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);

