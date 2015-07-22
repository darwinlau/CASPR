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
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, dynObj);
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
