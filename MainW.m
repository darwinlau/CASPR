% Load configs
clc; clear; close all; warning off;
%% Set up folders
% Change this
% Darwin's Computer
% folder = 'D:\Darwin''s Notebook Documents\Work\Research\Studies\Cable-driven manipulators\Simulations\Kinematics_dynamics';


% Jonathan's Home computer
% folder = 'C:\Users\Eden\Dropbox\mcdm-analysis.matlab';
% addpath(genpath('C:\Users\Eden\Dropbox\mcdm-analysis.matlab'));

% Jonathan's Laptop
% addpath(genpath('/home/jonathan/Dropbox/mcdm-analysis.matlab'))
% folder = '/home/jonathan/Dropbox/mcdm-analysis.matlab';

% Jonathan's Uni Computer
addpath(genpath('C:\Users\jpeden\Dropbox\mcdm-analysis.matlab'))
folder = 'C:\Users\jpeden\Dropbox\mcdm-analysis.matlab';
subfolder = 'systems_prop';

if(isunix)
    dlm = '/';
else
    dlm = '\';
end

%% Load the model - Only the models below have been tested for worksapce generation
% 2R Model - 4 Cables
model_folder        =   '2R_model';
cable_file          =   '2R_ideal_cable_props.csv';
body_file           =   '2R_body_prop.csv';

% 2R Model - 3 Cables
% model_folder        =   '2R_model_3C';
% cable_file          =   '2R_ideal_cable_props.csv';
% body_file           =   '2R_body_prop.csv';


% Ball and Socket Model - WCW at 0
% model_folder        =   'ball_socket_model';
% cable_file          =   'ball_socket_ideal_cable_props.csv';
% body_file           =   'ball_socket_body_prop.csv';

% Ball and Socket Model - No WCW at 0
% model_folder        =   'ball_socket_model_test';
% cable_file          =   'ball_socket_ideal_cable_props.csv';
% body_file           =   'ball_socket_body_prop.csv';

cables_prop_filepath = [folder,dlm,subfolder,dlm,model_folder,dlm,cable_file];
bodies_prop_filepath = [folder,dlm,subfolder,dlm,model_folder,dlm,body_file];

% Constructor for bodies and cables
bkConstructor = @() SystemKinematicsBodiesRigid(bodies_prop_filepath);
ckConstructor = @() SystemKinematicsCablesIdeal(cables_prop_filepath);
bdConstructor = @() SystemDynamicsBodiesRigid(bodies_prop_filepath);
cdConstructor = @() SystemDynamicsCablesIdeal(cables_prop_filepath);
% add SystemKinematicsTask, SystemDynamicsTask
% Construct the kinematics
sdConstructor = @() SystemDynamics(bdConstructor, cdConstructor, bkConstructor, ckConstructor);

%% Define the workspace condition
% Workspace conditions
wcondition  =   WorkspaceWrenchClosure(0);
% wcondition  =   TestDriftClosure();
% wcondition  =   PositiveControlContinuous();
% wcondition  =   WorkspaceTaskWrenchClosure();
% wcondition = WorkspaceStub();

%% Start the simulation
disp('Start Setup Simulation');
start_tic       =   tic;
wsim            =   WorkspaceSimulator(wcondition);
q_step          =   pi/36;
uGrid           =   UniformGrid(-pi*ones(2,1),(pi-q_step)*ones(2,1),q_step*ones(2,1));
% uGrid            =   UniformGrid([-97.5*pi/180;96.5*pi/180],[-96.5*pi/180;97.5*pi/180],[pi/720;pi/720]);
% uGrid            =   UniformGrid(-pi*ones(3,1),(pi-pi/9)*ones(3,1),pi/9*ones(3,1));
% uGrid            =   UniformGrid(-5*pi/9*ones(2,1),-4*pi/9*ones(2,1),pi/18*ones(2,1));
% grid = TaskGrid(-pi*ones(2,1),-pi*ones(2,1),-2,2,0.05);
time_elapsed    =   toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

disp('Start Running Simulation');
start_tic       =   tic;
wsim.run(uGrid, sdConstructor);
% [adjacency_matrix,laplacian_matrix] = wsim.toAdjacencyMatrix();
% con_comp = wsim.findConnectedComponents(adjacency_matrix);
time_elapsed    =   toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

disp('Start Plotting Simulation');
start_tic = tic;
wsim.plotWorkspace();
% wsim.plotWorkspaceComponents(con_comp);
time_elapsed = toc(start_tic);
fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);

% [x,y] = meshgrid(-180:180,-180:180);
% for i = 1:length(x)
% for j = 1:length(y)
% z(i,j) = cosd(x(i,j)) + cosd(x(i,j)+y(i,j));
% end
% end
% contour(x,y,z,-2:0.1:2)
