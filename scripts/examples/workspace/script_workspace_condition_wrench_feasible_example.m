% Script file for wrench feasible workspace
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
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/10;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Define the workspace condition and metrics
i_max     = 2^(modelObj.numDofs)-1;
w_set     = zeros(modelObj.numDofs,i_max+1);
flag_vec  = zeros(modelObj.numDofs,1);
min_vec     = -ones(modelObj.numDofs,1);
max_vec     = ones(modelObj.numDofs,1);
for k = 0:i_max
    flag_set = dec2bin(k,modelObj.numDofs);
    flag_vec(:) = str2num(flag_set(:)); %#ok<ST2NM>
    w_set(:,k+1) = min_vec.*(~flag_vec) + max_vec.*flag_vec;
end
w_condition     =   {WorkspaceConditionBase.CreateWorkspaceCondition(WorkspaceConditionType.WRENCH_FEASIBLE,[],w_set)};
w_metrics       =   {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.CONDITION_NUMBER,[])};
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);
opt             =   PointWorkspaceSimulatorOptions(false,optimset('Display','off'));

% Start the simulation
disp('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[],w_connectivity);

% Plot the simulation
disp('Start Plotting Simulation');
wsim.plotWorkspaceGraph();