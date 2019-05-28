% Script file for generating the wrench-closure workspace (point-based)
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clc; clear; warning off; close all;

% CASPR_configuration.SetDevModelConfig(1)
model_config    =   DevModelConfig('CU-Brick');
cable_set_id    =   'demo_causewaybay';

% Set up the model 
% model_config    =   ModelConfig('The Cable Robot Simulator');
% cable_set_id    =   'normal';

modelObj        =   model_config.getModel(cable_set_id);

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/3;
% Set up the workspace simulator
% Defne the grid
% q_begin(4:end,:) = zeros(3,1) ;
% q_end(4:end,:) = zeros(3,1) ;
uGrid           =   UniformGrid(q_begin, q_end, q_step,'step_size');
% Define the workspace condition(s): must have at least 1 condition
w_conditions    =   {WrenchClosureCondition([])};
% Define the workspace metric(s) (optional)
w_metrics       =   {ConditionNumberMetric()};
% Define the connectivity condition for point-wise workspaces
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj, uGrid, w_conditions, w_metrics, w_connectivity);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run();
%% Simulation result operation -- Plotting
% Graph plot
graph_plot = wsim.workspace.plotGraph(w_conditions,w_metrics,w_connectivity);

% Workspace plot
% plot specific axis with specified variable value (2D/3D plot)
close all force
plot_axis = [1 2 3];
fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [1 3 2 0 0 0];
cartesian_workspace_plot = wsim.workspace.plotWorkspace(plot_axis, w_conditions, [], fixed_variables);


% 2D/3D slider plot 
close all force
sliding_axis = [3];% 1 sliding axis only
plot_axis = [1 2 3];
fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [1 3 2 0 0 0];
cartesian_workspace_plot_slide = wsim.workspace.plotWorkspaceSlider(plot_axis,sliding_axis,w_conditions, w_metrics, fixed_variables);

% Workspace opertation
% construct workspace for by metric(s)/condition(s)
wsim.workspace.createWorkspaceGraph(w_conditions, w_metrics, w_connectivity)

% filter workspace by metric value
close all force
metric_value_min = [0.1 0.26];
metric_value_max = [];
filter_workspace = wsim.workspace.fliterWorkspaceMetric(w_metrics,metric_value_min,metric_value_max);
cartesian = filter_workspace.plotWorkspace(plot_axis, w_conditions, w_metrics, fixed_variables);


