% Load configs
clc; clear; warning off; %close all;
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
subfolder = 'data\config\models';

if(isunix)
    dlm = '/';
else
    dlm = '\';
end

%% 2R_planar Model 4 Cables
model_folder 	= 	'2R_planar';
cable_file 	 	= 	'2R_planar_cable_properties.xml';
body_file 	 	= 	'2R_planar_body_properties.xml';

%% Set up the file paths
cable_prop_filepath = [folder,dlm,subfolder,dlm,model_folder,dlm,cable_file];
body_prop_filepath = [folder,dlm,subfolder,dlm,model_folder,dlm,body_file];

body_xmlobj = XmlOperations.XmlReadRemoveIndents(body_prop_filepath);
cable_xmlobj = XmlOperations.XmlReadRemoveIndents(cable_prop_filepath);

%% Initialisation
dynObj = SystemKinematicsDynamics.LoadXmlObj(body_xmlobj, cable_xmlobj);
% Define the workspace condition
% Workspace conditions
% wcondition = WorkspaceStub();
% wcondition  =   WorkspaceWrenchClosure();
% wcondition  =   PositiveControlContinuous();
wcondition  =   WorkspaceStaticClosure();
% wcondition  =   WorkspaceStatic();
% wcondition  =   WorkspaceTaskWrenchClosure();
%% Define the metric
metric = NullMetric();
% metric = UnilateralDexterityMetric();
% metric = TensionFactorMetric();
% metric = SemiSingularMetric();
% metric = RelativeVolumeMetric();
% metric = RelativeRadiusMetric([0;0]);
% metric = CapacityMarginMetric();
% metric = CapacityMarginAccelerationMetric();
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
% wsim.boundaryFilter();
% [adjacency_matrix,laplacian_matrix] = wsim.toAdjacencyMatrix();
% con_comp = wsim.findConnectedComponents(adjacency_matrix);
time_elapsed    =   toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);

disp('Start Plotting Simulation');
start_tic = tic;
wsim.plotWorkspace();
% wsim.plotWorkspacePlane();
% wsim.plotFilterWorkspace();
% wsim.plotWorkspaceComponents(con_comp);
% time_elapsed = toc(start_tic);
% fprintf('End Plotting Simulation : %f seconds\n', time_elapsed);

% [x,y] = meshgrid(-180:180,-180:180);
% for i = 1:length(x)
% for j = 1:length(y)
% z(i,j) = cosd(x(i,j)) + cosd(x(i,j)+y(i,j));
% end
% end
% contour(x,y,z,-2:0.1:2)
