% Load configs
clc; clear; warning off; close all;
%% Temporarily add libraries to the current path
%  ASK IF THIS SHOULD BE KEPT
path_string = fileparts(mfilename('fullpath'));
path_string = path_string(1:strfind(path_string, 'scripts')-2);
addpath(genpath(path_string));

%% Initialise objects
model_config = ModelConfig(ModelConfigType.M_2R_PLANAR_XZ);
cable_set_id = 'basic_4_cables';
bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);


%% Initialisation
dynObj = SystemKinematicsDynamics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);
% Define the workspace condition
% Workspace conditions
% wcondition = WorkspaceStub();
% wcondition  =   WorkspaceWrenchClosure();
% wcondition  =   PositiveControlContinuous();
% wcondition  =   WorkspaceStaticClosure();
wcondition  =   WorkspaceStatic();
% wcondition  =   WorkspaceTaskWrenchClosure();
% wcondition = Example2Workspace()
%% Define the metric
% metric = NullMetric();
% metric = UnilateralDexterityMetric();
% metric = TensionFactorMetric();
% metric = SemiSingularMetric();
% metric = RelativeVolumeMetric();
% metric = RelativeRadiusMetric([0;0]);
% metric = CapacityMarginMetric();
metric = CapacityMarginAccelerationMetric();
% metric = MagnitudeVelocityMetric();


%% Start the simulation
disp('Start Setup Simulation');
start_tic       =   tic;
wsim            =   WorkspaceSimulator(dynObj,wcondition,metric);
q_step          =   pi/18;
n_dim           =   2;
uGrid           =   UniformGrid(-pi*ones(n_dim,1),(pi-q_step)*ones(n_dim,1),q_step*ones(n_dim,1));
% uGrid           =   UniformGrid(-[pi*ones(n_dim-1,1);q_step/5],[(pi-q_step)*ones(n_dim-1,1);q_step/5],[q_step*ones(n_dim-1,1);q_step/5]);
% uGrid           =   UniformGrid([97*pi/180;pi/6],[97*pi/180+2*q_step;pi/6+2*q_step],q_step*ones(n_dim,1));
% uGrid           =   UniformGrid(-pi/2*ones(n_dim,1),pi/2*ones(n_dim,1),q_step*ones(n_dim,1));
% uGrid           =   UniformGrid([pi-3*q_step;-pi],(pi-q_step)*ones(n_dim,1),q_step*ones(n_dim,1));
% uGrid = UniformGrid([pi/4;0;0],[pi/4;pi/4;pi/4;],[pi/4;pi/4;pi/4;])
% uGrid = UniformGrid([89*pi/180;17*pi/18],[pi/2;pi-q_step],q_step*ones(n_dim,1));
% grid = TaskGrid(-pi*ones(2,1),-pi*ones(2,1),-2,2,0.05);
time_elapsed    =   toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

disp('Start Running Simulation');
start_tic       =   tic;
wsim.run(uGrid);
time_elapsed    =   toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

disp('Start Plotting Simulation');
start_tic = tic;
wsim.plotWorkspace();
time_elapsed = toc(start_tic);
fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);
