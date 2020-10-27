% Script file for generating the interference of quad surf 
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clear all;close all;clc
model_config    =   ModelConfig('Example spatial'); 
cable_set_id    =   'cross_8_cables';
modelObj        =   model_config.getModel(cable_set_id);
modelObj =  model_config.getModel('cross_8_cables', [], ModelModeType.COMPILED);

q_begin         =   modelObj.bodyModel.q_min; 
q_begin(4:6) = 0;
% q_begin(1:3) = 0.5;
q_end = modelObj.bodyModel.q_max; 
q_end(4:6) = 0;
% q_end(1:3) = 0.5;
%% for rotational first
% q_begin = [0.2 0 0.7 0 0 0]';
% q_end = [0.2 1 0.7 0 0 0]';
% q_begin = [0.35 0.9 0.3 0 0 0]';
% q_end = [0.35 0.9 0.3 pi/2 0 0]';
% q_begin = [0.1 0.4 0.55 -pi/1.1 0 0]';
% q_end = [0.1 0.4 0.55 pi/2 0 0]';
% q_begin = [0.1 0.4 0.55 0 -pi/2 0 ]';
% q_end = [0.1 0.4 0.55 0 0 0]';
% q_begin = [0.1 0.35 0.55  0 0 -pi/2 ]';
% q_end = [0.1 0.35 0.55  0 0 pi/2]';

q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/20;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Workspace settings and conditions

min_segment_percentage = 1;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE_CABLE_CABLE,min_segment_percentage,modelObj)};

w_metrics = {};

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =    RayWorkspaceSimulator(modelObj,uGrid,w_condition,w_metrics);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run()

% %% optional functions (not yet tested)
% 
% % Plot point graph
% wsim.workspace.plotPointGraph(w_metrics);
% % % Plot ray graph
% wsim.workspace.plotRayGraph(w_metrics);
% % 
% plot_axis = [1 2 3];
% fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [1 3 2 0 0 0];
% % % % Plot point workspace
% % plot_1 = wsim.workspace.plotPointWorkspace(plot_axis,fixed_variables)
% % plot_1.SizeData = 1.5
% % 
% % % Plot ray workspace
% plot_2 = wsim.workspace.plotRayWorkspace(plot_axis,fixed_variables)
