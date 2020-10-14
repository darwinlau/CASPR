%%
clear all
clc
close all
%%
model_config    =   ModelConfig('Example spatial'); 
% cable_set_id    =   'cross_8_cables';
% modelObj        =   model_config.getModel(cable_set_id);
modelObj =  model_config.getModel('cross_8_cables', [], ModelModeType.COMPILED);

% model_config = ModelConfig('BMArm');
% modelObj = model_config.getModel('multi_link', [], ModelModeType.COMPILED); % Compiled mode needs to fix the plotting TODO

%%
q_begin   =   modelObj.bodyModel.q_min; 
q_end     =   modelObj.bodyModel.q_max; 
q_begin(4:6) = 0;
q_end(4:6) = 0;
% q_begin(1:3) = [0.5 0.2 0.2]';
% q_end(1:3) = [0.5 0.2 0.2]';
% q_begin = [0 0.4 0 0 0 0]';
% q_end = [1 0.4 0 0 0 0]';
% q_begin = [0.5 0.2 0.2 0 0 -pi/2]';
% q_end = [0.5 0.2 0.2 0 0 pi/2]';
% q_begin = [0.5 0.2 0.2 -1 -1 -0.5]';
% q_end = [0.5 0.2 0.2 1 -1 -0.5]';
q_step          =   (modelObj.bodyModel.q_max - modelObj.bodyModel.q_min)/20;
uGrid           =   UniformGrid(q_begin,q_end,q_step,'step_size');

%% obstacle
% tree
% F_i_xyz{3} = @(x,y,z) (x-0.5).^2 + (y-0.5).^2 - 0.002;
% faceSide(3) = -1;
% boundary{3} = [0 1 0 1 0 0.1];
% % 
% F_i_xyz{2} = @(x,y,z) (x-0.5).^2/0.0035 + (y-0.5).^2 /0.0035 - (z - 0.5).^2/0.05
% boundary{2} = [0 1 0 1 0.25 0.5];
% faceSide(3) = -1;
% F_i_xyz{1} = @(x,y,z) (x-0.5).^2/0.0045 + (y-0.5).^2 /0.0045 - (z - 0.35).^2/0.05
% boundary{1} = [0 1 0 1 0.1 0.35];
% faceSide(1) = -1;
% % 
% QuadSurfBoundary_equ = [];
%%

F_i_xyz{1} = @(x,y,z) (x - 0.5)^2 + (y - 0.25)^2 + (z - 0.5)^2 - 0.03
boundary{1} = [0.25 0.75 0.0 0.75 0.25 0.75];
% F_i_xyz{2} = @(x,y,z) (x - 0.5)^2 + (y - 0.35)^2 + (z - 0.5)^2 - 0.03
% boundary{2} = [0.25 0.75 0.0 0.75 0.25 0.75];
% QuadSurfBoundary_equ = [];
% F_i_xyz{1} = @(x,y,z) (x - 0)^2 + (y - 0.1)^2 + (z - 0.3)^2 - 0.005;
% boundary{1} = [-0.1 0.1 -0.1 0.2 -0.5 0.5];
faceSide(1) = -1;
QuadSurfBoundary_equ = [];

%%% donut/bean for bmarm
% R = 0.1;
% r = R/4;
% obs=@(x,y,z) ((150*x)^2 + (150*(y-0.25))^2 + (150*z)^2 + R^2 - r^2)^2 - 4*R^2*((150*x)^2 + (150*(y-0.25))^2)
% F_i_xyz{1} =@(x,y,z) ((x)^2 + ((y-0.3))^2 + (z)^2 + R^2 - r^2)^2 - 4*R^2*((x)^2 + ((y-0.25))^2);
% ha = fimplicit3(F_i_xyz{1} ,'FaceAlpha',1,'MeshDensity',35,'EdgeColor','none');
% QuadSurf.boundary{1} = [ha.XRange ha.YRange ha.ZRange];

%%% donut/bean for single link
% R = 0.2;
% r = R/4;
% F_i_xyz{1} =@(x,y,z) ((x).^2 + ((y-0.2)).^2 + (z - 0.4).^2 + R.^2 - r.^2).^2 - 4*R.^2.*((x).^2 + ((y-0.2)).^2);
% ha = fimplicit3(F_i_xyz{1} ,'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
% boundary{1} = [0 0.25 0 0.45 0.35 0.45];
% faceSide(1) = -1;
% QuadSurfBoundary_equ = [];

%% box
F_i_xyz{1} =@(x,y,z) 1*(x-0) + 0*(y-0) + 0*(z-0) ;
boundary{1} = [-0.0001 0.0001 0.25 0.55 0 0.15];
faceSide(1) = 1;
F_i_xyz{2} =@(x,y,z) 1*(x-0.3) + 0*(y-0) + 0*(z-0) ;
boundary{2} = [0.29999 0.30001 0.25 0.55 0 0.15];
faceSide(2) = -1;

F_i_xyz{3} =@(x,y,z) 0*(x-0) + 1*(y-0.25) + 0*(z-0) ;
boundary{3} = [0 0.3 0.24999 0.25001 0 0.15];
faceSide(3) = 1;
F_i_xyz{4} =@(x,y,z) 0*(x-0) + 1*(y-0.55) + 0*(z-0) ;
boundary{4} = [0 0.3 0.54999 0.55001 0 0.15];
faceSide(4) = -1;

F_i_xyz{5} =@(x,y,z) 0*(x-0) + 0*(y-0) + 1*(z-0) ;
boundary{5} = [0 0.3 0.25 0.55 -0.0001 0.0001];
faceSide(5) = 1;
F_i_xyz{6} =@(x,y,z) 0*(x-0) + 0*(y-0) + 1*(z-0.15) ;
boundary{6} = [0 0.3 0.25 0.55 0.14999 0.15001];
faceSide(6) = -1;

QuadSurfBoundary_equ{1} =@(t) [0;0.25;0.0 + 0.15*t];
QuadSurfBoundary_equ{2} =@(t) [0;0.55;0.0 + 0.15*t];
QuadSurfBoundary_equ{3} =@(t) [0;0.3*t+0.25;0.0];
QuadSurfBoundary_equ{4} =@(t) [0;0.3*t+0.25;0.15];

QuadSurfBoundary_equ{5} =@(t) [0.3;0.25;0.0 + 0.15*t];
QuadSurfBoundary_equ{6} =@(t) [0.3;0.55;0.0 + 0.15*t];
QuadSurfBoundary_equ{7} =@(t) [0.3;0.3*t+0.25;0.0];
QuadSurfBoundary_equ{8} =@(t) [0.3;0.3*t+0.25;0.15];

QuadSurfBoundary_equ{9} =@(t) [0+0.3*t;0.25;0];
QuadSurfBoundary_equ{10} =@(t) [0+0.3*t;0.25;0.15];
QuadSurfBoundary_equ{11} =@(t) [0.0+0.3*t;0.55;0.15];
QuadSurfBoundary_equ{12} =@(t) [0.0+0.3*t;0.55;0];


obstacle  = CreateObstacleElement(F_i_xyz,boundary,faceSide,QuadSurfBoundary_equ);

%%
figure
for i = 1:size(F_i_xyz,2)
ha(i)= fimplicit3(F_i_xyz{i},boundary{i},'FaceColor',[0.2,0.2,0.2],'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
hold on
end
Boundary_equ_degree = [1 1 1 1 1 1 1 1 1 1 1 1];
surface_degree = [1 1 1 1 1 1];

%% draw robot
% xlim([0 1])
% ylim([0 1])
% zlim([0 1])
t = linspace(0,1,20);

for i = 1:size(t,2)
    modelObj.update(q_begin + t(i)*(q_end - q_begin),zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1),zeros(modelObj.numDofs,1));
    [ee_graph, cable_graph] = draw_robot(modelObj,q_begin + t(i)*(q_end - q_begin));
    if i == 40
    
    else
        delete(ee_graph);
    end
%     pause(0.2)
%     delete(cable_graph(1));
%     delete(cable_graph(3));
%     delete(cable_graph(4));
%     delete(cable_graph(5));
%     delete(cable_graph(2));
%     delete(cable_graph(7));
%     delete(cable_graph(6));
%     delete(cable_graph(8));
end
%%
for k = 1:size(F_i_xyz,2) 
QuadSurf.Implicit_Function{k} = F_i_xyz{k};
QuadSurf.Surface_degree(k)= surface_degree(k);
end

for i = 1:size(QuadSurfBoundary_equ,2)    
QuadSurf.Boundary_equ{i} = QuadSurfBoundary_equ{i};
QuadSurf.Boundary_equ_degree(i) = Boundary_equ_degree(i);

end
%%
min_segment_percentage = 1;
w_condition     =   {WorkspaceRayConditionBase.CreateWorkspaceRayCondition(WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF,min_segment_percentage,modelObj,obstacle)};

w_metrics = {};

ws_obs            =    RayWorkspaceSimulator(modelObj,uGrid,w_condition,w_metrics);

ws_obs.run()
ws_obs.compTime

plot_axis = [1 2 3];
fixed_variables = ws_obs.grid.q_begin' + ws_obs.grid.delta_q' .* [0 0 0 0 0 0];
% % % Plot point workspace
% plot_1 = wsim.workspace.plotPointWorkspace(plot_axis,fixed_variables)
% plot_1.SizeData = 1.5
% % 
% % % Plot ray workspace
% ws_obs.workspace.plotRayWorkspace(plot_axis,fixed_variables)
