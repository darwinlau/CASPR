% Script file for static workspace
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clc; clear; warning off; close all;

% Set up the model 
model_config    =   ModelConfig('IPAnema 1');
cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);

q_begin         = [-2 -1.5 0 0 0 0]'; q_end = [2 1.5 2 0 0 0]';
% q_begin and q_end are the workspace margin that we are interested to know 
q_step          = abs(q_end - q_begin)/10;
% q_step is how you discretise the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');

% Define the workspace conditions, metrics and connectivity condition
w_condition     =   {WorkspaceConditionBase.CreateWorkspaceCondition(WorkspaceConditionType.STATIC,[],[])};
w_metrics       =   {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.CONDITION_NUMBER,[])};
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);
opt             =   PointWorkspaceSimulatorOptions(false,optimset('Display','off'));
% Start the simulation
disp('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[],w_connectivity);

%% Plot the simulation
disp('Start Plotting Simulation');
 wsim.plotWorkspace3([],WorkspaceConditionType.STATIC,[1 2 3]);  
 % 1 2 3 represent the axes of 1 2 and 3
 % Other conditions or metric can be plotted also