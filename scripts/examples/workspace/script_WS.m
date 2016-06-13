% Script file to show how to use the inverse dynamics simulator
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :

% Load configs
clc; clear; warning off; close all;

% Set up the model 
model_type = ModelConfigType.M_2DOF_VSD;
cable_set_id = 'basic_vsd';
model_config = ModelConfig(model_type);
modelObj = model_config.getModel(cable_set_id);

% Define the workspace condition
wcondition  =   WorkspaceStatic('quad_prog');
% Define the metric
metric = SEACM();

% Start the simulation
disp('Start Setup Simulation');
wsim            =   WorkspaceSimulator(modelObj,wcondition,metric);
q_step          =   0.1;
n_dim           =   2;
uGrid           =   UniformGrid(0.1*ones(n_dim,1),0.9*ones(n_dim,1),q_step*ones(n_dim,1));

% Run the simulation
disp('Start Running Simulation');
wsim.run(uGrid);

% Plot the simulation
disp('Start Plotting Simulation');
wsim.plotWorkspace([],[]);
