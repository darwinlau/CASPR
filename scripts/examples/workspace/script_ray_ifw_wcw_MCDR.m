
clc;  close all; warning off; clear all;

% Set up the model  
% 'BMArm_X2'
segment_number = 5;
model_config    =   ModelConfig('BMArm_X4_2'); nsegvar = [segment_number segment_number segment_number segment_number]';

cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);
% modelObj        =   model_config.getModel(cable_set_id,ModelModeType.COMPILED);

q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; 
uGrid           =   UniformGrid(q_begin,q_end,(q_end-q_begin)./(nsegvar-1),'step_size');

% Workspace settings and conditions
%% Cable-EndEffector
% box = [1 2; 1 5; 1 10; 2 5; 3 4; 3 6; 3 12; 4 6; 5 7; 5 9; 5 10; 6 7; 6 11; 6 12];
% % X4-1
% box = [1 4; 2 3; 2 5; 2 10; 3 5; 4 6; 4 8; 5 6; 5 9; 5 10];
% % X4-2
% box = [1 4; 2 3; 2 5; 2 10; 3 5; 4 6; 4 8; 5 6; 5 9; 5 10];

% ds = 0.02;
% optt = {box; ds};
% rayCondType1 = WorkspaceRayConditionType.INTERFERENCE_C_E;
rayCondType2 = WorkspaceRayConditionType.WRENCH_CLOSURE;
% w_condition     =
% {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(rayCondType1,0.4,optt)7
%                     WorkspaceRayConditionBase.CreateWorkspaceRayCondition(rayCondType2,20,modelObj)};
w_condition     =  {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(rayCondType2,20,modelObj)};
opt             =   RayWorkspaceSimulatorOptions(false,false);

% Start the simulation
disp('Start Setup Simulation');
wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[])
wsim.plotGraph();
% wsim.plotRayWorkspace([1,2,3])
