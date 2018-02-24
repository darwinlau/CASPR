
clc;  close all; warning off; clear all;

% Set up the model  
% model_config    =   ModelConfig('2 DoF VSD');   %  2 DoF VSD  spatial7cable      BMArm_paper
% cable_set_id    =   'basic';
% modelObj        =   model_config.getModel(cable_set_id);
% nsegvar= [25;25];      % number of discritization on each axis. if the user desire to ignore discritization on one axis its corresponding discritiaztion number can be set to zero
 
% 'spatial7cable' 
% model_config    =   DevModelConfig('spatial7cable'); nsegvar= [3, 3, 3, 6, 6, 6]';
% '4_4_CDPR_planar'
model_config    =   DevModelConfig('4_4_CDPR_planar'); nsegvar = [25 25 25]';
% % 'MickMultiIFW'
% model_config    =   DevModelConfig('MickMultiIFW'); nsegvar = [2 6 6 6]';

cable_set_id    =   'original';
% modelObj        =   model_config.getModel(cable_set_id);
modelObj        =   model_config.getModel(cable_set_id,ModelModeType.COMPILED);


q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; 
uGrid           =   UniformGrid(q_begin,q_end,(q_end-q_begin)./(nsegvar-1),'step_size');
% Workspace settings and conditions
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.WRENCH_CLOSURE,2)};
w_metrics       =   {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.TENSION_FACTOR)};
% w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE,2)};
opt             =   RayWorkspaceSimulatorOptions(false,false);
% Start the simulation
disp('Start Setup Simulation');
wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,w_metrics)
wsim.plotGraph();
