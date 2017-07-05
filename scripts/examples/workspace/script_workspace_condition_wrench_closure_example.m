% Script file to show how to use the inverse dynamics simulator
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :

% Load configs
clc; clear; warning off; close all;

% Set up the model 
model_config    =   ModelConfig('2 DoF VSD');
cable_set_id    =   'basic_vsd';
modelObj        =   model_config.getModel(cable_set_id);

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/10;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step);
% Define the workspace condition and metrics
w_condition     =   {WorkspaceConditionBase.CreateWorkspaceCondition(WorkspaceConditionType.WRENCH_CLOSURE,[],[])};
opt             =   WorkspaceSimulatorOptions(false,optimset('Display','off'));

% Start the simulation
disp('Start Setup Simulation');
wsim            =   WorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[]);

% Plot the simulation
disp('Start Plotting Simulation');
wsim.plotWorkspace2([],WorkspaceConditionType.WRENCH_CLOSURE,[1,2]);