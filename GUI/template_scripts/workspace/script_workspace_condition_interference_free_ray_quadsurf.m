% Script file for generating the interference of quad surf 
%
% Author        : Autogenerate
% Created       : 20XX
% Description    :

% Load configs
clear all;close all;clc
model_config    =   ModelConfig('Example spatial'); 
cable_set_id    =   'cross_8_cables';
modelObj        =   model_config.getModel(cable_set_id);
% modelObj =  model_config.getModel('cross_8_cables', [], ModelModeType.COMPILED);

q_begin         =   modelObj.bodyModel.q_min; 
q_begin(4:6) = 0;
% q_begin(1:3) = 0.5;
q_end = modelObj.bodyModel.q_max; 
q_end(4:6) = 0;
% q_end(1:3) = 0.5;
%% for rotational first
% q_begin = [0.2 0 0.7 0 0 0]';
% q_end = [0.2 1 0.7 0 0 0]';
% q_begin = [0.35 0.9 0.3 0 0 0]';
% q_end = [0.35 0.9 0.3 pi/2 0 0]';
% q_begin = [0.1 0.4 0.55 -pi/1.1 0 0]';
% q_end = [0.1 0.4 0.55 pi/2 0 0]';
% q_begin = [0.1 0.4 0.55 0 -pi/2 0 ]';
% q_end = [0.1 0.4 0.55 0 0 0]';
% q_begin = [0.1 0.35 0.55  0 0 -pi/2 ]';
% q_end = [0.1 0.35 0.55  0 0 pi/2]';

q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/20;
% Set up the workspace simulator
% First the grid
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');
% Workspace settings and conditions


%% Sphere
% QuadSurfImplicitEqu{1} = @(x,y,z) (x-1)^2+(y-0.5)^2+(z-0.6)^2 -0.2*x-0.5*y+0.2*z+0.4*x*y+0.2*y*z-0.3*x*z+0.2
% QuadSurf.boundary = [-0.1500 1.4706 0 1 0 1];

%% Ball
% QuadSurfImplicitEqu{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.5)^2 + (z - 0.5)^2 - 0.05
% QuadSurf.boundary{1} = [0.25 0.75 0.25 0.75 0.25 0.75];

QuadSurfImplicitEqu{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.25)^2 + (z - 0.5)^2 - 0.03
QuadSurf.boundary{1} = [0.25 0.75 0.0 0.75 0.25 0.75];
QuadSurfImplicitEqu{2} = @(x,y,z) (x - 0.5)^2 + (y - 0.35)^2 + (z - 0.5)^2 - 0.03
QuadSurf.boundary{2} = [0.25 0.75 0.0 0.75 0.25 0.75];
% % % 
% QuadSurfImplicitEqu{2} = @(x,y,z) (x - 0.45)^2 + (y - 0.45)^2 + (z - 0.5)^2 - 0.03
% QuadSurf.boundary{2} = [0.25 0.75 0.25 0.75 0.25 0.75];
% QuadSurfImplicitEqu{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.5)^2 + (z - 0.5)^2 - 1
% QuadSurf.boundary{1} = [-1 2 -1 2 -1 2];

% % 
% QuadSurfImplicitEqu{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.5)^2 - 0.25
% QuadSurf.boundary{1} = [-1 2 -1 2 0.7 0.7];

% QuadSurfImplicitEqu{2} = @(x,y,z) (x - 0.5)^2 + (y - 0.5)^2 + (z - 0.1)^2 - 0.03
% QuadSurf.boundary{2} = [0.25 0.75 0.25 0.75 -0.2 0.4];

%% Cylinder           
% QuadSurfImplicitEqu{3} = @(x,y,z) (x-0.5)^2 + (y-0.75)^2 - 0.02;
% QuadSurf.boundary{3} = [0 1 0 1 0 1];
% 
% QuadSurfImplicitEqu{3} = @(x,y,z) (x-0.5)^2 + (y-0.5)^2 - 0.002;
% QuadSurf.boundary{3} = [0 1 0 1 0 0.1];
%% Cone
% QuadSurfImplicitEqu{1} = @(x,y,z) (x-0.5)^2/0.005 + (y-0.5)^2 /0.005 - z^2/0.05
% QuadSurf.boundary{1} = [0 1 0 1 0 1];

% QuadSurfImplicitEqu{1} = @(x,y,z) (x-0.5)^2/0.0035 + (y-0.5)^2 /0.0035 - (z - 0.5)^2/0.05
% QuadSurf.boundary{1} = [0 1 0 1 0.25 0.5];
% 
% QuadSurfImplicitEqu{2} = @(x,y,z) (x-0.5)^2/0.0045 + (y-0.5)^2 /0.0045 - (z - 0.35)^2/0.05
% QuadSurf.boundary{2} = [0 1 0 1 0.1 0.35];

% Random flat quad surface
% QuadSurfImplicitEqu{2} = @(x,y,z) (0.01.*x+0.5).^2 + (0.02.*y-0.5).^2 - z.^2 -0.4;
% QuadSurf.boundary{2} = [0.45 0.75 0.45 0.75 0 1];

%%
% clf
for i = 1:size(QuadSurfImplicitEqu,2)
% obs(i) = fimplicit3(QuadSurfImplicitEqu{i},QuadSurf.boundary{i},'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.8);
obs(i) = fimplicit3(QuadSurfImplicitEqu{i},QuadSurf.boundary{i},'FaceAlpha',0.9,'MeshDensity',35,'EdgeColor','none');
hold on;
end
% xlim([0 1])
% ylim([0 1])
% zlim([0 1])
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
% %     delete(cable_graph(6));
% %     delete(cable_graph(8));
% end

% s = sym('s%d',[1 10]);
% QuadSurfImplicitEqu{1} = @(x,y,z) s(1)*x^2 + s(2)*y^2 + s(3)*z^2 + ...
%                                       s(4)*x*y + s(5)*x*z + s(6)*z*y + ...
%                                       s(7)*x + s(8)*y + s(9)*z + s(10);
% Q = s;
% QuadSurf.Quad_Matrix{1} = [Q(1) 0.5*Q(4) 0.5*Q(6) 0.5*Q(7);
%     0.5*Q(4) Q(2) 0.5*Q(5) 0.5*Q(8);
%     0.5*Q(6) 0.5*Q(5) Q(3) 0.5*Q(9)
%     0.5*Q(7) 0.5*Q(8) 0.5*Q(9) Q(10)];

%%

%% in the form of xHx^T = 0
for k = 1:size(QuadSurfImplicitEqu,2)
syms x y z;
Q = zeros(1,10);
[coeff_f,var_f] = coeffs(QuadSurfImplicitEqu{k}(x,y,z));
for i = 1:size(coeff_f,2)
    if isequal(var_f(i),x^2)
        Q(1) = coeff_f(i);
    elseif isequal(var_f(i),y^2)
        Q(2) = coeff_f(i);
    elseif isequal(var_f(i),z^2)
        Q(3) = coeff_f(i);
    elseif isequal(var_f(i),x*y)
        Q(4) = coeff_f(i);
    elseif isequal(var_f(i),x*z)
        Q(5) = coeff_f(i);
    elseif isequal(var_f(i),y*z)
        Q(6) = coeff_f(i);
    elseif isequal(var_f(i),x)
        Q(7) = coeff_f(i);
    elseif isequal(var_f(i),y)
        Q(8) = coeff_f(i);
    elseif isequal(var_f(i),z)
        Q(9) = coeff_f(i);
    else
        Q(10) = coeff_f(i);
    end
    
end

QuadSurf.Quad_Matrix{k} = [Q(1) 0.5*Q(4) 0.5*Q(6) 0.5*Q(7);
    0.5*Q(4) Q(2) 0.5*Q(5) 0.5*Q(8);
    0.5*Q(6) 0.5*Q(5) Q(3) 0.5*Q(9)
    0.5*Q(7) 0.5*Q(8) 0.5*Q(9) Q(10)];
end

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
% wsim.workspace.plotPointGraph(w_metrics);
% % % Plot ray graph
% wsim.workspace.plotRayGraph(w_metrics);
% % 
% plot_axis = [1 2 3];
% fixed_variables = wsim.grid.q_begin' + wsim.grid.delta_q' .* [1 3 2 0 0 0];
% % % Plot point workspace
% plot_1 = wsim.workspace.plotPointWorkspace(plot_axis,fixed_variables)
% plot_1.SizeData = 1.5
% % 
% % % Plot ray workspace
% wsim.workspace.plotRayWorkspace(plot_axis,fixed_variables)
