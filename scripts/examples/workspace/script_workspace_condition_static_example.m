% Script file for static workspace
%
% Author        : Arthur Chan
% Created       : 2018
% Description   : A simple example on showing how to plot workspace

% Load configs
clear; warning off; close all; clc;

% Set up the model 
model_config    =   ModelConfig('IPAnema 1');
cable_set_id    =   'original';
modelObj        =   model_config.getModel(cable_set_id);

q_begin         =   [-2 -1.5 0 -0.5 -0.5 -0.1]'; q_end = [2 1.5 2 0.5 0.5 0.1]';
% q_begin and q_end are the workspace margin that we are interested to know
q_step          =   abs(q_end - q_begin)./[10 10 10 4 4 2]';
% q_step is how you discretise the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');

% Define the workspace conditions, metrics and connectivity condition
w_condition     =   {WorkspaceConditionBase.CreateWorkspaceCondition(WorkspaceConditionType.STATIC,[],[])};
w_metrics       =   {WorkspaceMetricBase.CreateWorkspaceMetric(WorkspaceMetricType.CONDITION_NUMBER,[])};
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);
opt             =   PointWorkspaceSimulatorOptions(false,optimset('Display','off'));
% Start the simulation
disp('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj,uGrid,opt);

% Run the simulation
disp('Start Running Simulation');
wsim.run(w_condition,[],w_connectivity);

%% Plot the simulation
close all;
disp('Start Plotting Simulation');
% Syntax: wsim.plotWorkspaceSlide3([],plot_axis,capability_measure,slices,fixed_dim_cor,slide_dim_index)
% input the dimension that is needed into be plotted to 'slices'
% input the coordinates of fixed axis into 'fixed_dim_cor', in ascending order
% input the index of sliding axis into 'slide_dim_index'

% Example 1: 2D plot with axis 1 and 2 be the plotting axes and the coordinate of axis 3,4,5,6 is fixed at [1 0 0 0]
wsim.plotWorkspace2([],WorkspaceConditionType.STATIC,[1 2],[1 0 0 0]');

% Example 2: 2D slider plot with axis 2 and 3 be the plotting axes and the axis 4 be the sliding axis,
% coordinate of axis 1,5,6 is fixed at [-0.25 0 0]
wsim.plotWorkspaceSlide2([],WorkspaceConditionType.STATIC,[2 3],[-0.4 0 0]',[4]);

% Example 3: 3D plot with axis 1,2,3 be the plotting axes and the coordinate of axis 4,5,6 is fixed at [0 0 0]
wsim.plotWorkspace3([],WorkspaceConditionType.STATIC,[4 5 6],[0 0 0]');

% Example 4: 2D slider plot with axis 1,2,3 be the plotting axes and the axis 3 be the sliding axis,
% coordinate of axis 5,6 is fixed at [0 0]
wsim.plotWorkspaceSlide3([],WorkspaceConditionType.STATIC,[1 2 3],[0 0]',[4]);
    
 % 1 2 3 represent the axes of 1 2 and 3
 % Other conditions or metric can be plotted also