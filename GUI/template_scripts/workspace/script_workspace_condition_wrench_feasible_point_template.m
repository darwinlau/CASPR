% Script file for generating the wrench-feasible workspace (point-based)
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clc; clear; warning off; close all;

% Set up the model 
model_config    =   ModelConfig('Example planar XY');
cable_set_id    =   'basic';
modelObj        =   model_config.getModel(cable_set_id);
% Define the number of points per axis
num_seg = 5;
q_begin         =   modelObj.bodyModel.q_min; q_end = modelObj.bodyModel.q_max;
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/num_seg;
% Set up the workspace simulator
% Defne the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Define the WFW desired wrench set
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
% Define the workspace condition(s): must have at least 1 condition
w_conditions    =   {WrenchFeasibleCondition(w_set, [])};
% Define the workspace metric(s) (optional)
w_metrics       =   {};
% Define the connectivity condition for point-wise workspaces
w_connectivity  =   WorkspaceConnectivityBase.CreateWorkspaceConnectivityCondition(WorkspaceConnectivityType.GRID,uGrid);

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =   PointWorkspaceSimulator(modelObj, uGrid, w_conditions, w_metrics, w_connectivity);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run();

%% Simulation result operation -- Plotting

% Workspace plot
% plot specific axis with specified variable value (2D/3D plot)
% Graph plot
graph_plot = wsim.workspace.plotGraph(w_conditions,w_metrics,w_connectivity);

figure
plot_axis = [1 2 3];% Maximum allow 3 axis plot e.g. here 1st, 2nd variables as the axis
% Fixed variables, you can leave the value of plot axis zero/any number, it won't affect the result plot
% 4 digits numbers are counted into the plotting error
fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [0 0 0];
% 2D/3D plot
cartesian_workspace_plot = wsim.workspace.plotWorkspace(plot_axis, w_conditions, w_metrics, fixed_variables);

% close all 
% % 2D/3D slider plot 
% sliding_axis = [3];% Slider axis maximum 1 input so far
% 
% cartesian_workspace_plot_slide = wsim.workspace.plotWorkspaceSlider(plot_axis,sliding_axis,w_conditions, w_metrics, fixed_variables);
