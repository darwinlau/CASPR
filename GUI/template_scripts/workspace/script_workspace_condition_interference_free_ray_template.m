% Script file for generating the interference free workspace (ray-based)
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
% Workspace settings and conditions
min_segment_percentage = 20;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE,min_segment_percentage,modelObj)};
opt             =   RayWorkspaceSimulatorOptions(false,false);

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run(w_condition,[])

% Plot the simulation
CASPR_log.Info('Start Plotting Simulation');
wsim.plotGraph();