% Script file for generating the wrench-closure workspace (point-based)
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

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/5;
% Set up the workspace simulator
% Defne the grid
uGrid           =   UniformGrid(q_begin, q_end, q_step,'step_size');
% Define the workspace condition(s): must have at least 1 condition
w_conditions    =   {WrenchClosureCondition([])};
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
wsim.plotGraph();