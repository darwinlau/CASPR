% Script file to show the example on interference free workspace 
%
% Author        : Zeqing Zhang
% Created       : 2017
% Description   : example on IFW

% Load configs
clc; clear; close all;

% Set up the model 
model_config = ModelConfig('Example planar XY'); 
cable_set_id = 'basic_notouch';
modelObj = model_config.getModel(cable_set_id);

% Set up the workspace simulator
% First the grid
q_step          =  0.1 ; n_dim           = 3;
uGrid           =   UniformGrid([0.5; 0.5; 0], [0.5; 0.5; pi], [0; 0; pi/10],'step_size');

% Define the workspace condition and metrics
w_condition =   {WorkspaceConditionBase.CreateWorkspaceCondition(WorkspaceConditionType.INTERFERENCE,[],[])};
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);
opt = PointWorkspaceSimulatorOptions(false,optimset('Display','off'));

% Start the simulation
disp('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition, [], w_connectivity);

% % Plot the simulation
% disp('Start Plotting Simulation');
wsim.plotWorkspaceGraph();