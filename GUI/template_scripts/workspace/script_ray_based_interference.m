%%
% clear all
clc
% close all

% model_config    =   ModelConfig('Example spatial'); 
% cable_set_id    =   'cross_8_cables';
% modelObj        =   model_config.getModel(cable_set_id);
% modelObj =  model_config.getModel('cross_8_cables', [], ModelModeType.COMPILED);

model_config = ModelConfig('BMArm');
modelObj = model_config.getModel('multi_link', [], ModelModeType.COMPILED); % Compiled mode needs to fix the plotting TODO

%%
q_begin   =   modelObj.bodyModel.q_min; 
q_end     =   modelObj.bodyModel.q_max; 
q_begin(4:6) = 0;
q_end(4:6) = 0;
% q_begin(1:3) = [0.5 0.2 0.25]'; % cable-box-Goursat
% q_end(1:3) = [0.5 0.2 0.25]';  % cable-box-Goursat
% q_begin(1:3) = [0.5 0.4 0.3]'; % cable-box
% q_end(1:3) = [0.5 0.4 0.3]';  % cable-box
% q_begin(1:3) = [0.8 0.4 0.5]'; % cable-tree/orientation figure result
% q_end(1:3) = [0.8 0.4 0.5]';  % cable-tree/orientation figure result
% q_begin(1:3) = [0.5 0.3 0.25]'; % cable-tree/orientation figure result
% q_end(1:3) = [0.5 0.3 0.25]';  % cable-tree/orientation figure result
% 
% q_begin(1:3) = [0.8 0.4 0.1]'; % cable-torus
% q_end(1:3) = [0.8 0.4 0.1]';  % cable-torus
% q_begin(1:3) = [0.7 0.4 0.7]'; % cable-cubic 
% q_end(1:3) = [0.7 0.4 0.7]';  % cable-cubic
q_begin = [-1 0.5 0 0.8]'; %bmarm
q_end = [1 0.5 0 0.8]';    %bmarm
% q_begin = [-1 -1 0 0.8]'; %bmarm
% q_end = [1 1 0 0.8]';    %bmarm

% bug fixing
% q_begin = [0.8000 0.4000 0.1000 -0.933333333333333 -1.0000 -0.533333333333333 ]'; 
% q_end =  [0.8000 0.4000 0.1000 -0.933333333333333 1.0000 -0.533333333333333 ]'; 
%% 
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/40;
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');


%% tree
% F_i_xyz{1} = @(x,y,z) (x-0.5).^2 + (y-0.5).^2 - 0.008; %cylinder
% boundary{1} = [0.42 0.58 0.42 0.58 0 0.1];
% boundary{1} = [0 1 0 1 0 0.25];
% faceSide(1) = -1;
% 
% F_i_xyz{2} = @(x,y,z) (x-0.5).^2/0.3 + (y-0.5).^2 /0.3 - ((0.85*z) - 0.45).^2%lower
% boundary{2} = [0.35 0.65 0.35 0.65 0.20 0.45];
% faceSide(2) = -1;
% 
% F_i_xyz{3} = @(x,y,z) (x-0.5).^2/0.2 + (y-0.5).^2 /0.2 - ((0.85*z) - 0.6).^2 % upper
% boundary{3} = [0.35 0.65 0.35 0.65 0.4 0.7];
% faceSide(3) = -1;
% 
% QuadSurfBoundary_equ = [];
% Boundary2SurfaceIndex = [];
%%Ball
% F_i_xyz{1} =@(x,y,z)  (x-0.5).^2 + (y-0.5).^2 + (z-0.5).^2 - 0.2^2;
% faceSide(1) = -1;
% QuadSurfBoundary_equ = [];
% boundary{1} = [0.3 0.7 0.3 0.7 0.3 0.7];
% figure
% for i = 1:size(F_i_xyz,2)
% ha(i)= fimplicit3(F_i_xyz{i},'FaceColor',[0.2,0.2,0.2],'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
% hold on
% % xlim([0 1]);ylim([0 1]);zlim([0 1])
% end

%%

% F_i_xyz{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.25)^2 + (z - 0.5)^2 - 0.03
% boundary{1} = [0.25 0.75 0.0 0.75 0.25 0.75];
% F_i_xyz{2} = @(x,y,z) (x - 0.5)^2 + (y - 0.35)^2 + (z - 0.5)^2 - 0.03
% boundary{2} = [0.25 0.75 0.0 0.75 0.25 0.75];
% QuadSurfBoundary_equ = [];
% F_i_xyz{1} = @(x,y,z) (x - 0)^2 + (y - 0.1)^2 + (z - 0.3)^2 - 0.005;
% boundary{1} = [-0.1 0.1 -0.1 0.2 -0.5 0.5];
% faceSide(1) = -1;
% QuadSurfBoundary_equ = [];

%% donut/bean for bmarm
% R = 0.015;
% r = R/4;
% % obs=@(x,y,z) ((150*x)^2 + (150*(y-0.25))^2 + (150*z)^2 + R^2 - r^2)^2 - 4*R^2*((150*x)^2 + (150*(y-0.25))^2)
% F_i_xyz{1} =@(x,y,z) 0.7*(((x)^2 + ((1.5*y-0.4))^2 + (z)^2 + R^2 - r^2)^2 - 4*R^2*((2.*x)^2 + ((y-0.4))^2));
% boundary{1} = [-0.18 0.18 0.25 0.5 -0.1 0.1];
% faceSide(1) = -1;
% QuadSurfBoundary_equ = [];
%% ball for bmarm
F_i_xyz{1} =@(x,y,z)  (x-0.0).^2 + (y-0.35).^2 + (z-0.0).^2 - 0.08^2;
faceSide(1) = -1;
QuadSurfBoundary_equ = [];
boundary{1} = [0-0.08 0+0.08 0.35-0.08 0.35+0.08 0-0.08 0+0.08 ];
 Boundary2SurfaceIndex = [];
% figure
% for i = 1:size(F_i_xyz,2)
% ha(i)= fimplicit3(F_i_xyz{i},'FaceColor',[0.2,0.2,0.2],'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
% hold on
% % xlim([0 1]);ylim([0 1]);zlim([0 1])
% end

% xlim([-0.4 0.4]);ylim([-0.4 0.4]);zlim([-0.4 0.4])
%% donut/bean for single link
% R = 0.2;
% r = R/4;
% F_i_xyz{1} =@(x,y,z) ((x-0.3).^2 + ((y-0.4)).^2 + (z - 0.4).^2 + R.^2 - r.^2).^2 - 4*R.^2.*((x-0.3).^2 + ((y-0.4)).^2);
% faceSide(1) = -1;
% QuadSurfBoundary_equ = [];
% boundary{1} = [0.05 0.55 0.15 0.65 0.35 0.45];
% Boundary2SurfaceIndex = [];
%% Goursat
% a = 0;
% b = 0;
% c = -0.0005;
% F_i_xyz{1} =@(x,y,z) (x -0.15).^4 + (y - 0.4).^4 + 17*(z - 0.075).^4 + a*((x - 0.15).^2 +(y - 0.4).^2 +(z-0.075).^2) + c;
% F_i_xyz{1} =@(x,y,z) (x -0.15).^4 + (y - 0.4).^4 + (z - 0.4).^4 + a*((x - 0.15).^2 +(y - 0.4).^2 +(z-0.075).^2) + c;
% boundary{1} = [0 0.3 0.24 0.56 0.25 0.55];
% % ha = fimplicit3(F_i_xyz{1} ,'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
% faceSide(1) = -1;
% QuadSurfBoundary_equ = [];
% Boundary2SurfaceIndex = [];
%%
% a = 2;
% F_i_xyz{1} =@(x,y,z) (1.22*(x-0.8)).*((x-0.8).^2 - 3*(y-0.5).^2) - 0.5*(z-0.5)
% boundary{1} = [0.3 0.7 0.3 0.7 0.4 0.5];
% ha = fimplicit3(F_i_xyz{1},boundary{1},'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
% faceSide(1) = -1;
% QuadSurfBoundary_equ = [];
% Boundary2SurfaceIndex = [];3

%% box
% F_i_xyz{1} =@(x,y,z) 1*(x-0) + 0*(y-0) + 0*(z-0) ;
% boundary{1} = [-0.0001 0.0001 0.25 0.55 0.25 0.55];
% faceSide(1) = 1;
% F_i_xyz{2} =@(x,y,z) 1*(x-0.3) + 0*(y-0) + 0*(z-0) ;
% boundary{2} = [0.29999 0.30001 0.25 0.55 0.25 0.55];
% faceSide(2) = -1;
% 
% F_i_xyz{3} =@(x,y,z) 0*(x-0) + 1*(y-0.25) + 0*(z-0) ;
% boundary{3} = [0 0.3 0.24999 0.25001 0.25 0.55];
% faceSide(3) = 1;
% F_i_xyz{4} =@(x,y,z) 0*(x-0) + 1*(y-0.55) + 0*(z-0) ;
% boundary{4} = [0 0.3 0.54999 0.55001 0.25 0.55];
% faceSide(4) = -1;
% 
% F_i_xyz{5} =@(x,y,z) 0*(x-0) + 0*(y-0) + 1*(z-0.25) ;
% boundary{5} = [0 0.3 0.25 0.55 0.24999 0.2501];
% faceSide(5) = 1;
% F_i_xyz{6} =@(x,y,z) 0*(x-0) + 0*(y-0) + 1*(z-0.55) ;
% boundary{6} = [0 0.3 0.25 0.55 0.54999 0.55001];
% faceSide(6) = -1;
% 
% QuadSurfBoundary_equ{1} =@(t) [0; 0.25; 0.25 + 0.3*t];
% QuadSurfBoundary_equ{2} =@(t) [0; 0.55; 0.25 + 0.3*t];
% QuadSurfBoundary_equ{3} =@(t) [0; 0.3*t+0.25; 0.25];
% QuadSurfBoundary_equ{4} =@(t) [0; 0.3*t+0.25; 0.55];
% 
% QuadSurfBoundary_equ{5} =@(t) [0.3; 0.25; 0.25 + 0.15*t];
% QuadSurfBoundary_equ{6} =@(t) [0.3; 0.55; 0.55 + 0.15*t];
% QuadSurfBoundary_equ{7} =@(t) [0.3; 0.3*t+0.25; 0.25];
% QuadSurfBoundary_equ{8} =@(t) [0.3; 0.3*t+0.25; 0.55];
% 
% QuadSurfBoundary_equ{9} =@(t) [0+0.3*t; 0.25; 0.25];
% QuadSurfBoundary_equ{10} =@(t) [0+0.3*t; 0.25; 0.55];
% QuadSurfBoundary_equ{11} =@(t) [0.0+0.3*t; 0.55; 0.55];
% QuadSurfBoundary_equ{12} =@(t) [0.0+0.3*t; 0.55; 0.25];
% 
% Boundary2SurfaceIndex = [1  1  3;
%                          2  1  4;
%                          3  1  5;
%                          4  1  6;
%                          5  2  3;
%                          6  2  4;
%                          7  2  5;
%                          8  2  6;
%                          9  3  5;
%                         10  3  6;
%                         11  4  6;
%                         12  4  5];
% 
%%


%%
% figure
% for i = 1:size(F_i_xyz,2)
% ha(i)= fimplicit3(F_i_xyz{i},boundary{i},'FaceColor',[0.2,0.2,0.2],'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
% hold on
% % xlim([0 1]);ylim([0 1]);zlim([0 1])
% xlim([-0.4 0.4]);ylim([-0.4 0.4]);zlim([-0.4 0.4])
% end

%% draw robot
% xlim([0 1])
% ylim([0 1])
% zlim([0 1])
% 
% t = linspace(0,1,20);
% 
% for i = 1:size(t,2)
%     modelObj.update(q_begin + t(i)*(q_end - q_begin),zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1),zeros(modelObj.numDofs,1));
%     [ee_graph, cable_graph] = draw_robot(modelObj,q_begin + t(i)*(q_end - q_begin));
%     if i == 40
%     
%     else
%         delete(ee_graph);
%     end
% %     pause(0.2)
% %     delete(cable_graph(1));
% %     delete(cable_graph(3));
% %     delete(cable_graph(4));
% %     delete(cable_graph(5));
% %     delete(cable_graph(2));
% %     delete(cable_graph(7));
% %     delete(cable_graph(6));
% %     delete(cable_graph(8));
% end
%%
obstacle  = CreateObstacleElement(F_i_xyz,boundary,faceSide,QuadSurfBoundary_equ,Boundary2SurfaceIndex);

min_segment_percentage = 1;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF,min_segment_percentage,modelObj,obstacle)};

w_metrics = {};

ws_obs            =    RayWorkspaceSimulator(modelObj,uGrid,w_condition,w_metrics);

ws_obs.run()
ws_obs.compTime

%% BMarm
tmp_val  = ws_obs.workspace.rays{1}.intervals(:);
tmp_val(1) = -0.6;
tmp_val(end) = 0.6;
q_intersected = repmat(q_begin,1,size(tmp_val,1));
q_intersected(1,:) = tmp_val;
figure
obstacle.plotObstacle
for i = 1:size(tmp_val,1)
    draw_robot(modelObj,q_intersected(:,i));
end

% plot_axis = [1 2];
% fixed_variables = ws_obs.grid.q_begin' + ws_obs.grid.delta_q' .* [0 0 0 0];
% ws_fig = ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)
%%
% plot_axis = [4 5];
% plot_axis = [1 2 3];

% % % Plot point workspace
% plot_1 = wsim.workspace.plotPointWorkspace(plot_axis,fixed_variables)
% plot_1.SizeData = 1.5
% % 
% % % Plot ray workspace

%% BMARM
% figure
plot_axis = [1 2];
 fixed_variables = ws_sim.grid.q_begin' + ws_sim.grid.delta_q' .* [1 1 0 0];
ws_fig = ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)

%%


close all
figure
fixed_variables = ws_obs.grid.q_begin' + ws_obs.grid.delta_q' .* [0 0 0 0 0 1];
ws_fig = ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)
x0=100;
y0=100;
width=300;
height=300;
set(gcf,'position',[x0,y0,width,height])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom 0.98*ax_width 0.98*ax_height];   

figure
fixed_variables = ws_obs.grid.q_begin' + ws_obs.grid.delta_q' .* [0 0 0 0 0 7];
ws_fig = ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)
x0=100;
y0=100;
width=300;
height=300;
set(gcf,'position',[x0,y0,width,height])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom 0.98*ax_width 0.98*ax_height];

figure
fixed_variables = ws_obs.grid.q_begin' + ws_obs.grid.delta_q' .* [0 0 0 0 0 13];
ws_fig = ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)
x0=100;
y0=100;
width=300;
height=300;
set(gcf,'position',[x0,y0,width,height])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom 0.98*ax_width 0.98*ax_height];
figure
fixed_variables = ws_obs.grid.q_begin' + ws_obs.grid.delta_q' .* [0 0 0 0 0 19];
ws_fig = ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)
x0=100;
y0=100;
width=300;
height=300;
set(gcf,'position',[x0,y0,width,height])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom 0.98*ax_width 0.98*ax_height];
figure
fixed_variables = ws_obs.grid.q_begin' + ws_obs.grid.delta_q' .* [0 0 0 0 0 25];
ws_fig = ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)
x0=100;
y0=100;
width=300;
height=300;
set(gcf,'position',[x0,y0,width,height])
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom 0.98*ax_width 0.98*ax_height];

% hold on 
% obstacle.plotObstacle