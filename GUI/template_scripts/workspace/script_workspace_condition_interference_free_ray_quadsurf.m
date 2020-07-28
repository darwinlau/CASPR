% Script file for generating the interference of quad surf 
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clear all;close all;clc

model_config    =   ModelConfig('Example spatial'); 
cable_set_id    =   'basic_8_cables';
modelObj        =   model_config.getModel(cable_set_id);

q_begin         =   modelObj.bodyModel.q_min; 
q_begin(4:6) = 0;
% q_begin(1:3) = 0.5;
q_end = modelObj.bodyModel.q_max; 
q_end(4:6) = 0;
% q_end(1:3) = 0.5;
%% for rotational first
% q_begin = [0.3 0 0.5 0 0 0]';
% q_end = [0.3 1 0.5 0 0 0]';
% q_begin = [0.35 0.9 0.3 0 0 0]';
% q_end = [0.35 0.9 0.3 pi/2 0 0]';
% q_begin = [0.1 0.2 0.55 0 -pi/2 0]';
% q_end = [0.1 0.2 0.55 0 pi/2 0]';

q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/3;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Workspace settings and conditions


%% Sphere
% QuadSurf.implicit_equation{1} = @(x,y,z) (x-1)^2+(y-0.5)^2+(z-0.6)^2 -0.2*x-0.5*y+0.2*z+0.4*x*y+0.2*y*z-0.3*x*z+0.2
% QuadSurf.boundary = [-0.1500 1.4706 0 1 0 1];

%% Ball
% QuadSurf.implicit_equation{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.5)^2 + (z - 0.5)^2 - 0.05
% QuadSurf.boundary = [0.25 0.75 0.25 0.75 0.25 0.75];
% 
% QuadSurf.implicit_equation{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.5)^2 + (z - 0.5)^2 - 0.03
% QuadSurf.boundary = [0.25 0.75 0.25 0.75 0.25 0.75];


%% Cylinder           
% QuadSurf.implicit_equation{1} = @(x,y,z) (x-0.5)^2 + (y-0.75)^2 - 0.02;
% QuadSurf.boundary{1} = [0 1 0 1 0 1];

%% Cone
QuadSurf.implicit_equation{1} = @(x,y,z) (x-0.5)^2/0.005 + (y-0.5)^2 /0.005 - z^2/0.05
QuadSurf.boundary{1} = [0 1 0 1 0 1];


%% Random flat quad surface
% QuadSurf.implicit_equation{1} = @(x,y,z) (0.01.*x+0.5).^2 + (0.02.*y-0.5).^2 - z.^2 -0.4;
% QuadSurf.boundary{1} = [0.45 0.75 0.45 0.75 0 1];

%%
clf
ob1 = fimplicit3(QuadSurf.implicit_equation{1},QuadSurf.boundary{1},'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.4);
hold on;

%%  debug use
% t = linspace(0,1,30);
% for i = 1:30
%     modelObj.update(q_begin + t(i)*(q_end - q_begin),zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1),zeros(modelObj.numDofs,1));
%     [ee_graph, cable_graph] = draw_robot(modelObj);
%     if i == 40
%     
%     else
%         delete(ee_graph);
%     end
% %     delete(cable_graph(1));
% %     delete(cable_graph(3));
% %     delete(cable_graph(4));
% %     delete(cable_graph(5));
% %     delete(cable_graph(2));
% %     delete(cable_graph(7));
% %     delete(cable_graph(8));
% end


% s = sym('s%d',[1 10]);
% QuadSurf.implicit_equation{1} = @(x,y,z) s(1)*x^2 + s(2)*y^2 + s(3)*z^2 + ...
%                                       s(4)*x*y + s(5)*x*z + s(6)*z*y + ...
%                                       s(7)*x + s(8)*y + s(9)*z + s(10);


%%
min_segment_percentage = 1;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF,min_segment_percentage,modelObj,QuadSurf)};

w_metrics = {};

% Start the simulation
CASPR_log.Info('Start Setup Simulation');
wsim            =    RayWorkspaceSimulator(modelObj,uGrid,w_condition,w_metrics);

% Run the simulation
CASPR_log.Info('Start Running Simulation');
wsim.run()

% %% optional functions (not yet tested)
% 
% % Plot point graph
% wsim.workspace.plotPointGraph(w_condition,w_metrics);
% % Plot ray graph
% wsim.workspace.plotRayGraph(w_metrics);
% 
% plot_axis = [1 2 3];
% fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [1 3 2 0 0 0];
% % Plot point workspace
% wsim.workspace.plotPointWorkspace(plot_axis,w_condition,fixed_variables)
% 
% % Plot ray workspace
% wsim.workspace.plotRayWorkspace(plot_axis,w_condition,fixed_variables)
