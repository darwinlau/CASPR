% Script file for generating the interference free workspace (point-based)
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clc; clear; warning off; close all;

% Set up the model 
% model_config    =   ModelConfig('Example planar XY');
% cable_set_id    =   'basic';
model_config    =   ModelConfig('The Cable Robot Simulator');
cable_set_id    =   'normal';
modelObj        =   model_config.getModel(cable_set_id);

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)./[5 5 5 Inf Inf Inf]';

% Set up the workspace simulator
% Specify any fixed value if wanted
q_begin(4:end,:) = zeros(3,1) ;
q_end(4:end,:) = zeros(3,1) ;
% Defne the grid
uGrid           =   UniformGrid(q_begin, q_end, q_step,'step_size');


% Minimum distance for interference free
min_epsilon_d   = 0.01;
% Define the workspace condition(s): must have at least 1 condition
% So far there are multiple condtions and metrics, will updated to a new file later
w_conditions    =   {WrenchClosureCondition([]),WorkspaceStaticCondition([]),InterferenceFreeCondition([], min_epsilon_d)};
% Define the workspace metric(s) (optional)
w_metrics       =   {TensionFactorMetric,ConditionNumberMetric};
% Define the connectivity condition for point-wise workspaces
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj, uGrid, w_conditions, w_metrics, w_connectivity);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run();

% Plot the simulation, accept single/multiple conditions/metrics input
CASPR_log.Info('Start Plotting Simulation');

graph_plot = wsim.workspace.plotGraph(w_conditions{1},w_metrics,w_connectivity);

plot_axis = [1 2 4];% Maximum allow 3 axis plot e.g. here 1st, 2nd variables as the axis
% Fixed variables, you can leave the value of plot axis zero/any number, it won't affect the result plot
% 4 digits numbers are counted into the plotting error
fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [1 3 2 0 0 0];
close all force
% 2D/3D plot
cartesian_workspace_plot = wsim.workspace.plotWorkspace(plot_axis, w_conditions, w_metrics, fixed_variables);

close all force
% 2D/3D slider plot 
sliding_axis = [3];% Slider axis maximum 1 input so far

cartesian_workspace_plot_slide = wsim.workspace.plotWorkspaceSlider(plot_axis,sliding_axis,w_conditions, w_metrics, fixed_variables);

% Find the workspace that do not meet the metrics requirment(allow multiple inputs), all the
% functions exist in the old workspace can work
close all force
metric_value_min = [0.1 0.26];
metric_value_max = [];
filter_workspace = wsim.workspace.fliterWorkspaceMetric(w_metrics,metric_value_min,metric_value_max);
cartesian = filter_workspace.plotWorkspace(plot_axis, w_conditions, w_metrics, fixed_variables);

