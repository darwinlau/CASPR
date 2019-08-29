% Script file for generating the static workspace (point-based)
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clc; clear; warning off; close all;

% Set up the model 
model_config    =   ModelConfig('Example planar XY');
cable_set_id    =   'basic';
modelObj        =   model_config.getModel(cable_set_id);
% Define the number of points per axis
num_seg = 5;
q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/num_seg;
% Set up the workspace simulator
% Defne the grid
uGrid           =   UniformGrid(q_begin, q_end, q_step,'step_size');
% Define the workspace condition(s): must have at least 1 condition
w_conditions    =   {WorkspaceStaticCondition(WorkspaceStaticMethodType.M_SEACM)};
% Define the workspace metric(s) (optional)
w_metrics       =   {};
% Define the connectivity condition for point-wise workspaces
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj, uGrid, w_conditions, w_metrics, w_connectivity);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run();

% Plot the simulation
CASPR_log.Info('Start Plotting Simulation');
% Plot the simulation
% CASPR_log.Info('Start Plotting Simulation');
% graph_plot = wsim.workspace.plotGraph(w_conditions,w_metrics,w_connectivity);

graph_plot = wsim.workspace.plotGraph(w_conditions,w_metrics,w_connectivity);

figure
plot_axis = [1 2 3];% Maximum allow 3 axis plot e.g. here 1st, 2nd variables as the axis
% Fixed variables, you can leave the value of plot axis zero/any number, it won't affect the result plot
% 4 digits numbers are counted into the plotting error
fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [0 0 0];
% 2D/3D plot
cartesian_workspace_plot = wsim.workspace.plotWorkspace(plot_axis, w_conditions, w_metrics, fixed_variables);

% close all 
% % 2D/3D slider plot 
% sliding_axis = [3];% Slider axis maximum 1 input so far
% 
% cartesian_workspace_plot_slide = wsim.workspace.plotWorkspaceSlider(plot_axis,sliding_axis,w_conditions, w_metrics, fixed_variables);
