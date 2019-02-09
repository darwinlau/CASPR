% Script file for wrench closure workspace
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clc; clear; warning off; close all;

% Set up the model 
model_config    =   ModelConfig('2 DoF VSD');
cable_set_id    =   'basic';
modelObj        =   model_config.getModel(cable_set_id);

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; 
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/5;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Workspace settings and conditions
min_segment_percentage = 20;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.WRENCH_CLOSURE,min_segment_percentage,modelObj)};
w_metrics       =   {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.TENSION_FACTOR)};
opt             =   RayWorkspaceSimulatorOptions(false,false);

% Start the simulation
disp('Start Setup Simulation');
wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[])

% Plot the simulation
disp('Start Plotting Simulation');
wsim.plotGraph();