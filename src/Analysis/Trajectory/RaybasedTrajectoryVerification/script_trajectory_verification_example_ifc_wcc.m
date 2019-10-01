% script on trajectory verification for cable interference free conditions
% (IFC) and wrench closure condtions (WCC)
clc;  
% close all;
warning off; 
% clear all;

%% the model  
% 'CDPR_7c_6Dof'
% model_config    =   DevModelConfig('CDPR_7c_6Dof'); 
% cable_set_id    =   'original';
% model_config    =   DevModelConfig('CU-Brick'); 
% cable_set_id    =   'demo_causewaybay';
modelObj        =   model_config.getModel(cable_set_id);


%% Trajectories
time_range = [0 1];

traj = TrajectorySet(6);
%% workspace conditions
% required distance threshold for IFC
% clear tsim
min_dist = 0.01;

t_condtions = {TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.ANALYTICAL,TrajectoryConditionType.WRENCH_CLOSURE,80,[]),...
    TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.ANALYTICAL,TrajectoryConditionType.CABLE_INTERFERENCE_FREE,80,min_dist)};

t_condtions = {TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL,TrajectoryConditionType.WRENCH_CLOSURE,80,[]),...
    TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL,TrajectoryConditionType.CABLE_INTERFERENCE_FREE,80,min_dist)};

% t_condtions = {TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.ANALYTICAL,TrajectoryConditionType.CABLE_INTERFERENCE_FREE,80,min_dist)};
% t_condtions = {TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL,TrajectoryConditionType.CABLE_INTERFERENCE_FREE,80,min_dist)};

% t_condtions = {TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.ANALYTICAL,TrajectoryConditionType.WRENCH_CLOSURE,80,[])};
% t_condtions = {TrajectoryConditionBase.CreateTrajectoryCondition(TrajectoryAnalysisMethodType.NUMERICAL_ANALYTICAL,TrajectoryConditionType.WRENCH_CLOSURE,80,[])  };


t_types = TrajectoryType.CONTROL_POINTS;
% t_types = TrajectoryTypeBase.CreateTrajectoryType(TrajectoryType.PARAMETRIC);
%% Start the simulation
disp('Start Setup Simulation');
tsim            =   TrajectoryWorkspaceSimulator(modelObj,t_condtions,t_types,traj,time_range);

disp('Start Running Simulation');
tsim.run()

%% plot the trajectory-based workspace

figure
pause(4)
for i = 1:size(tsim.workspace.trajectories,1)
% for i = 1:3
    grid on
    tsim.workspace.PlotAnimation(1)
% tsim.workspace.PlotTrajectory(i)
% pause(1.5)
% if i == 3
% clf
% end
end
