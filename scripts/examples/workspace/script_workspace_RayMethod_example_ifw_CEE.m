
clc;  close all; warning off; clear all;

% Set up the model  
% 'BMArm_X2'
segment_number = 7;
% model_config    =   DevModelConfig('4_4_CDPR_planar'); nsegvar = [segment_number segment_number segment_number]';
% model_config    =   DevModelConfig('Spatial_Tree'); nsegvar = [segment_number segment_number 0 0 0 0]';
model_config    =   DevModelConfig('BMArm_X2'); nsegvar = [segment_number segment_number segment_number 0]';

cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);
% modelObj        =   model_config.getModel(cable_set_id,ModelModeType.COMPILED);

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; 
% q_begin         =   [0.4;0;0;0;0;0]; q_end = [0.5;1;1;5*pi/180;5*pi/180;5*pi/180]; 
uGrid           =   UniformGrid(q_begin,q_end,(q_end-q_begin)./(nsegvar-1),'step_size');

% Workspace settings and conditions
%% Cable-cable
% w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE,0,modelObj)};
%% Cable-EndEffector
box = [5 7; 5 9; 5 10; 6 7; 6 11; 6 12; 1 10; 3 12];
ds = 0.02;
optt = {box; ds};
rayCondType = WorkspaceRayConditionType.INTERFERENCE_C_E;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(rayCondType,0,optt)};
%% Cable-Obstacle
% NEED TO BE FULLED

opt             =   RayWorkspaceSimulatorOptions(false,false);

% Start the simulation
disp('Start Setup Simulation');
wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[])
% wsim.run(w_condition,w_metrics)
% wsim.plotGraph();
wsim.plotRayWorkspace([1,2,3])
