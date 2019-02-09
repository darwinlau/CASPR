% Script file for wrench closure workspace
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
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/5;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Workspace settings and conditions
min_segment_percentage = 20;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE,min_segment_percentage,modelObj)};
opt             =   RayWorkspaceSimulatorOptions(false,false);

% Start the simulation
disp('Start Setup Simulation');
wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[])

% Plot the simulation
disp('Start Plotting Simulation');
wsim.plotGraph();



% % ifw_CEE: Interference Free Workspace between cables and the
% % end-effector(links)
% clc;  close all; warning off; clear all;
% 
% % Set up the model  
% % 'BMArm_X2'
% segment_number = 3;
% model_config    =   DevModelConfig('BMArm_X2'); nsegvar = [segment_number segment_number segment_number segment_number]';
% 
% cable_set_id    =   'original';
% modelObj        =   model_config.getModel(cable_set_id);
% % modelObj        =   model_config.getModel(cable_set_id,ModelModeType.COMPILED);
% 
% q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; 
% uGrid           =   UniformGrid(q_begin,q_end,(q_end-q_begin)./(nsegvar-1),'step_size');
% 
% % Workspace settings and conditions
% %% Cable-EndEffector
% % specify the cables and links to be investigated
% box = [5 7; 5 9; 5 10; 6 7; 6 11; 6 12; 1 10; 3 12];
% ds = 0.02;
% optt = {box; ds};
% rayCondType = WorkspaceRayConditionType.INTERFERENCE;
% w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(rayCondType,0,optt)};
% opt             =   RayWorkspaceSimulatorOptions(false,false);
% 
% % Start the simulation
% disp('Start Setup Simulation');
% wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);
% 
% % Run the simulation
% disp('Start Running Simulation');
% wsim.run(w_condition,[])
% % wsim.run(w_condition,w_metrics)
% % wsim.plotGraph();
% wsim.plotRayWorkspace([1,2,3])
