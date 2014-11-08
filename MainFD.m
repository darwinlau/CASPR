% Load configs
clc; clear; close all;
folder = 'D:\Darwin''s Notebook Documents\Work\Research\Cable-driven manipulators\Simulations\Kinematics_dynamics';

% Neck Model
% cables_attach_prop_file = [folder, '\systems_prop\8S_neck_model\8S_neck_cable_attachment_prop.csv'];
% cables_prop_file = [folder, '\systems_prop\8S_neck_model\8S_neck_cables_prop.csv'];
% bodies_prop_file = [folder, '\systems_prop\8S_neck_model\8S_neck_body_prop.csv'];
% trajec_prop_file = [folder, '\systems_prop\8S_neck_model\8S_neck_trajectory_gen.csv'];

% cables_attach_prop_file = [folder, '\systems_prop\SR_model\SR_attachment_prop.csv'];
% cables_prop_file = [folder, '\systems_prop\SR_model\SR_cables_prop.csv'];
% bodies_prop_file = [folder, '\systems_prop\SR_model\SR_body_prop.csv'];
% trajec_prop_file = [folder, '\systems_prop\SR_model\SR_trajectory2.csv'];

% cables_attach_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_cable_attachment_prop.csv'];
% cables_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_cables_prop.csv'];
% bodies_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_body_prop.csv'];
% trajec_prop_file = [folder, '\systems_prop\branched_6_link_model\branched_6_link_trajectory.csv'];

cables_attach_prop_file = [folder, '\systems_prop\planar_model\Planar_cable_attachment_prop.csv'];
cables_prop_file = [folder, '\systems_prop\planar_model\Planar_cables_prop.csv'];
bodies_prop_file = [folder, '\systems_prop\planar_model\Planar_body_prop.csv'];

disp('Start Setup Simulation');
start_tic = tic;
sim = ForwardDynamicsSimulator(bodies_prop_file, cables_attach_prop_file, cables_prop_file);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

disp('Start Running Simulation');
start_tic = tic;
time = 0:0.001:1;
cableforces = cell(1, length(time));
cableforces(:) = {[0; 0.5; 0.5; 0]};
sim.Run(cableforces, 0.01, 1, [0.3; 0.5; 0], [0; 0; 0])
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

sim.PlotJointSpace();
% disp('Start Plotting Simulation');
% start_tic = tic;
% plot_axis = [0 1 0 1 -0.1 0.1];
% %plot_axis = [-0.2 0.2 -0.2 0.2 -0.1 0.3];
% sim.PlotMovie(plot_axis, [folder, '\test.avi'], 2, 500, 640);
% time_elapsed = toc(start_tic);
% fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);

% 
% disp('Start Plotting Simulation');
% start_tic = tic;
% plot_axis = [0 1 0 1 -0.1 0.1];
% %plot_axis = [-0.2 0.2 -0.2 0.2 -0.1 0.3];
% sim.PlotMovie(plot_axis, [folder, '\test.avi'], 2, 500, 640);
% time_elapsed = toc(start_tic);
% fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
% 
% sim.PlotCableLengths();
% sim.PlotBodyCOG();