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

%idsolver = IDMinLinCableForce(ones(dynObj.numCables,1));
idsolver = IDMinQuadCableForce(ones(dynObj.numCables,1));
%idsolver = IDMinInteraction(ones(6*dynObj.numLinks,1));
%idsolver = IDMinQuadCableForcesConInteractionAngle(ones(dynObj.numCables,1), 15*pi/180*ones(dynObj.numCables,1));
%idsolver = IDMinInteractionConInteractionAngle(ones(6*dynObj.numLinks,1), 20*pi/180*ones(dynObj.numLinks, 1));
%idsolver = IDNullSpaceMinQuadCableForce(dynObj.numCables);
%idsolver = IDNullSpaceMinInteraction(dynObj.numCables);

disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(dynObj, idsolver);
trajectory = JointTrajectory.LoadXmlObj(trajectories_xmlobj.getElementById(trajectory_id), dynObj);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% Run solver
disp('Start Running Simulation');
start_tic = tic;
idsim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

fprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, mean iterations : %d\n', mean(idsim.compTime), std(idsim.compTime), round(mean(idsim.compIterations)));

% % Right muscles
% idsim.PlotCableForces(1:38); xlabel('time [s]'); ylabel('cable forces [N]'); title('Forces (right)');
% % Left muscles
% idsim.PlotCableForces(39:76); xlabel('time [s]'); ylabel('cable forces [N]'); title('Forces (left)');

% idsim.plotJointSpace();
% idsim.plotInteractionForceZ();
% idsim.plotInteractionForceMagnitudes();
% idsim.plotInteractionForceAngles();
% idsim.plotInteractionMomentMagnitudes();
%idsim.plotIDCost();
% idsim.verifyEoMConstraint();
% 
% disp('Start Plotting Simulation');
% start_tic = tic;
% plot_axis = [0 1 0 1 -0.1 0.1];
% %plot_axis = [-0.2 0.2 -0.2 0.2 0 0.3];
% idsim.plotMovie(plot_axis, [folder, '\test.avi'], 2, 500, 640);
% time_elapsed = toc(start_tic);
% fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
