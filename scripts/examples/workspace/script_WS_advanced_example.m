% Script file to show how to use the inverse dynamics simulator
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    :

% Load configs
clc; clear; close all;

% Set up the model 
model_type = ModelConfigType.M_2DOF_VSD;
cable_set_id = 'basic_vsd';
model_config = ModelConfig(model_type);
modelObj = model_config.getModel(cable_set_id);

% Set up the workspace simulator
% First the grid
q_step          =   0.05; n_dim           =   2;
uGrid           =   UniformGrid(0.05*ones(n_dim,1),0.45*ones(n_dim,1),q_step*ones(n_dim,1));
% Define the workspace condition and metrics
w_condition  =   {WorkspaceConditionBase.CreateWorkspaceCondition(WorkspaceConditionType.WRENCH_CLOSURE,[],[])};
w_metric = {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.SEACM,[])};
opt = WorkspaceSimulatorOptions(true);

% Start the simulation
disp('Start Setup Simulation');
wsim            =   WorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,w_metric);
wsim.run([],{WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.TENSION_FACTOR,[])});

% Plot the simulation
disp('Start Plotting Simulation');
wsim.plotWorkspace2([],WorkspaceConditionType.WRENCH_CLOSURE,[1,2]);
wsim.plotWorkspace2([],WorkspaceMetricType.TENSION_FACTOR,[1,2]);
wsim.filterWorkspaceMetric(WorkspaceMetricType.SEACM,-100,100);
wsim.plotWorkspace2([],WorkspaceMetricType.SEACM,[1,2]);