% Script file for generating the wrench-closure workspace (ray-based)
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
model_config    =   DevModelConfig('CU-Brick');
cable_set_id    =   'AEI_demo';
modelObj        =   model_config.getModel(cable_set_id);
% Set up the model 
% model_config    =   ModelConfig('Example planar XY');
% cable_set_id    =   'basic';

q_begin         =   modelObj.bodyModel.q_min; 
q_begin(4:6) = -pi/2;
q_begin = [0.5 0.5 0.5 -pi/2 -pi/2 -pi/2]';
q_end = modelObj.bodyModel.q_max; 
q_end(4:6) = pi/2;
q_end = [0.5 0.5 0.5 pi/2 pi/2 pi/2]';
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/3;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Workspace settings and conditions
QuadSurf = @(x,y,z) x.^2 + y.^2 + z.^2 - 0.2.*x - 1.*y + 0.1.*z + 0.5.*x.*y  + 0.2.*y.*z - 0.3.*x.*z+ 0.04;
QuadSurf = @(x,y,z) (x-1).^2 + (y+0.5).^2 + (z-0.2).^2 - 0.2.*x - 1.*y + 0.1.*z + 0.4.*x.*y  + 0.2.*y.*z - 0.3.*x.*z+ 0.04;
QuadSurf = @(x,y,z) (x-1).^2 + (y-0.5).^2 + (z-0.2).^2 - 0.2.*x - 0.5.*(y) + 0.1.*z + 0.4.*x.*y  + 0.2.*y.*z - 0.3.*x.*z+ .2;
delete(ob1)
ob1 = fimplicit3(QuadSurf,[-2 2 -2 2 0 2.5],'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.4)
            hold on;
            
% QuadSurf = @(x,y,z) x.^2 + y.^2 - 0.5;
% QuadSurf = @(x,y,z) (x-0.2).^2 + (y-0.8).^2 - 0.01;
% QuadSurf = @(x,y,z) (0.01.*x+0.5).^2 + (0.02.*y-0.5).^2 - z.^2 -0.4;
% QuadSurf = @(x,y,z) x.^2 + (y-0.1).^2 - 0.5;
% QuadSurf = @(x,y,z) (x - 1).^2 + y.^2 - 0.5;


min_segment_percentage = 1;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF,min_segment_percentage,modelObj,QuadSurf)};

% w_metrics       =   {TensionFactorMetric,ConditionNumberMetric};
% w_metrics       =   {ConditionNumberMetric()};
w_metrics = {};

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =    RayWorkspaceSimulator(modelObj,uGrid,w_condition,w_metrics);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run()
%% optional functions

% Plot point graph
wsim.workspace.plotPointGraph(w_condition,w_metrics);
% Plot ray graph
wsim.workspace.plotRayGraph(w_condition,w_metrics);

plot_axis = [1 2 3];
fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [1 3 2 0 0 0];
% Plot point workspace
wsim.workspace.plotPointWorkspace(plot_axis,w_condition,fixed_variables)

% Plot ray workspace
wsim.workspace.plotRayWorkspace(plot_axis,w_condition,fixed_variables)
