
clc;  close all; warning off; clear all;

%%%          2 DoF VSD

% Set up the model 
% 
model_config    =   DevModelConfig('4_4_CDPR_planar');   %    spatial7cable   BMArm_paper   BMArm_paper    '2 DoF VSD'
cable_set_id    =  'original' ;                         %     'basic'
modelObj        =   model_config.getModel(cable_set_id);


q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max; 
% nsegvar= [20;20;20;20];      % number of discritization on each axis. if the user desire to ignor discritization on one axis its corresponding discritiaztion number can be set to zero
nsegvar= [25;25;25];
uGrid           =   UniformGrid(q_begin,q_end,(q_end-q_begin)./(nsegvar-1),'step_size');
% Workspace settings and conditions
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.WRENCH_CLOSURE,2)};
opt             =   RayWorkspaceSimulatorOptions(false,false);
% Start the simulation
disp('Start Setup Simulation');
wsim            =   RayWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[]);
wsim.plotRayWorkspace



