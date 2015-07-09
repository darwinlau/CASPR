% Load configs
clc; clear; clear classes;% close all;
folder = 'D:\Darwin''s Notebook Documents\Work\Research\Studies\Cable-driven manipulators\Simulations\Kinematics_dynamics';

% Neck Model
cables_prop_filepath = [folder, '\systems_prop\8S_neck_model\8S_neck_ideal_cable_prop.csv'];
bodies_prop_filepath = [folder, '\systems_prop\8S_neck_model\8S_neck_body_prop.csv'];
trajec_prop_filepath = [folder, '\systems_prop\8S_neck_model\8S_neck_trajectory_roll.csv'];

%cables_prop_filepath = [folder, '\systems_prop\SR_model\SR_ideal_cable_props.csv'];
% cables_prop_filepath = [folder, '\systems_prop\SR_model\SR_ideal_cable_props_8_cables_2.csv'];
% bodies_prop_filepath = [folder, '\systems_prop\SR_model\SR_body_prop.csv'];
% %trajec_prop_filepath = [folder, '\systems_prop\SR_model\SR_trajectory.csv'];
% trajec_prop_filepath = [folder, '\systems_prop\SR_model\SR_trajectory2.csv'];

% cables_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_cables_prop.csv'];
% bodies_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_body_prop.csv'];
% trajec_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_trajectory.csv'];

% cables_prop_filepath = [folder, '\systems_prop\planar_model\planar_ideal_cable_props.csv'];
% bodies_prop_filepath = [folder, '\systems_prop\planar_model\planar_rigid_body_props.csv'];
% trajec_prop_filepath = [folder, '\systems_prop\planar_model\planar_trajectory.csv'];


bkConstructor = @() SystemKinematicsBodiesRigid(bodies_prop_filepath);
ckConstructor = @() SystemKinematicsCablesIdeal(cables_prop_filepath);
bdConstructor = @() SystemDynamicsBodiesRigid(bodies_prop_filepath);
cdConstructor = @() SystemDynamicsCablesIdeal(cables_prop_filepath);

sdConstructor = @() SystemDynamics(bdConstructor, cdConstructor, bkConstructor, ckConstructor);

sd0 = sdConstructor();

%idsolver = IDMinLinCableForce(ones(sd0.numCables,1));
idsolver = IDMinQuadCableForce(ones(sd0.numCables,1));
%idsolver = IDMinInteraction(ones(6*sd0.numLinks,1));
%idsolver = IDMinQuadCableForcesConInteractionAngle(ones(sd0.numCables,1), 15*pi/180*ones(sd0.numCables,1));
%idsolver = IDMinInteractionConInteractionAngle(ones(6*sd0.numLinks,1), 20*pi/180*ones(sd0.numLinks, 1));
%idsolver = IDNullSpaceMinQuadCableForce(sd0.numCables);
%idsolver = IDNullSpaceMinInteraction(sd0.numCables);

disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(idsolver);
trajectory = JointTrajectory.ReadFromConfig(trajec_prop_filepath, bkConstructor());
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% Run solver
disp('Start Running Simulation');
start_tic = tic;
idsim.run(trajectory, sdConstructor);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

fprintf('Optimisation computational time, mean : %f seconds, std dev : %f seconds, mean iterations : %d\n', mean(idsim.compTime), std(idsim.compTime), round(mean(idsim.compIterations)));

% % Right muscles
% idsim.PlotCableForces(1:38); xlabel('time [s]'); ylabel('cable forces [N]'); title('Forces (right)');
% % Left muscles
% idsim.PlotCableForces(39:76); xlabel('time [s]'); ylabel('cable forces [N]'); title('Forces (left)');

%idsim.plotJointSpace();
% %idsim.plotInteractionZForce();
% %idsim.plotInteractionMomentMagnitudes(); xlabel('time [s]'); ylabel('moment [N.m]');
% idsim.plotIDCost();
% idsim.verifyEoMConstraint();

% disp('Start Plotting Simulation');
% start_tic = tic;
% %plot_axis = [0 1 0 1 -0.1 0.1];
% plot_axis = [-0.2 0.2 -0.2 0.2 0 0.3];
% idsim.plotMovie(plot_axis, [folder, '\test.avi'], 2, 500, 640);
% time_elapsed = toc(start_tic);
% fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
