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

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/5;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Define the workspace condition and metrics
w_condition     =   {WorkspaceConditionBase.CreateWorkspaceCondition(WorkspaceConditionType.STATIC,[],[])};
w_metrics       =   {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.CONDITION_NUMBER,[])};
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);
opt             =   PointWorkspaceSimulatorOptions(false,optimset('Display','off'));

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run(w_condition,[],w_connectivity);

% Plot the simulation
CASPR_log.Info('Start Plotting Simulation');
wsim.plotGraph();