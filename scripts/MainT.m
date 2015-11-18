%This is a main file to test the new wrench set approximations.

%% Setup
% Load configs
clc; clear; warning off; close all;
%% Set up folders
% Change this
% Darwin's Computer
% folder = 'D:\Darwin''s Notebook Documents\Work\Research\Studies\Cable-driven manipulators\Simulations\Kinematics_dynamics';

% Jonathan's Home computer
folder = 'C:\Users\jpeden\Dropbox\mcdm-analysis.matlab';
% folder = 'C:\Users\Eden\Dropbox\mcdm-analysis.matlab';
addpath(genpath('C:\Users\jpeden\Dropbox\mcdm-analysis.matlab'));
% addpath(genpath('C:\Users\Eden\Dropbox\mcdm-analysis.matlab'));

% Jonathan's Laptop
addpath(genpath('/home/jonathan/Dropbox/mcdm-analysis.matlab'))
folder = '/home/jonathan/Dropbox/mcdm-analysis.matlab';

%% Initialise objects
model_config = ModelConfig(ModelConfigType.M_2R_PLANAR_XZ);
cable_set_id = 'basic_4_cables';

if(isunix)
    dlm = '/';
else
    dlm = '\';
end
bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);


%% Initialisation
dynObj = SystemKinematicsDynamics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

%% Setup the simulation
disp('Start Setup Simulation');
start_tic       =   tic;
% The pose to generate wrench set for
q = [-4*pi/3;pi/3];
% q = [-pi/2;pi/2];
time_elapsed    =   toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

%% Start the simulation
disp('Start Running Simulation');
start_tic       =   tic;
dynObj.update(q, zeros(size(q)), zeros(size(q)));
w = WrenchSet(dynObj.L,1.0*dynObj.cableDynamics.forcesMax,dynObj.cableDynamics.forcesMin);
w2 = WrenchSet(dynObj.L/dynObj.M,1.0*dynObj.cableDynamics.forcesMax,dynObj.cableDynamics.forcesMin);
w_ca = w.sphereApproximationCapacity(dynObj.G);
time_elapsed    =   toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);
% % m = UnilateralDexterityMetric();
% % m = TensionFactorMetric();
m = CapacityMarginMetric();
% % m = RelativeVolumeMetric();
% % m = RelativeRadiusMetric([0;0])
% % m = SemiSingularMetric()
% mv = m.evaluate(dynObj,0.2)
mv = m.evaluate(dynObj,1)
% mv
%% Plot the wrench sets
disp('Start Plotting Simulation');
start_tic = tic;
figure; hold on; grid on;
for i = 1:w.n_faces
    x = [-300,300];
    y = pinv(w.A(i,:))*w.b(i) + null(w.A(i,:)).*x;
    plot(y(1),y(2),'b')
end
axis([-3000,3000,-3000,3000])
% Plot the capacity margin approximation
theta = -pi:pi/100:pi;
x1 = w_ca.T(1) + w_ca.r*cos(theta);x2 = w_ca.T(2) + w_ca.r*sin(theta);
plot(x1,x2,'m')

plot(-50*dynObj.L(:,1),-50*dynObj.L(:,2),'go')

figure; hold on; grid on;
for i = 1:w.n_faces
    x = [-3000,3000];
    y = (1/w2.A(i,2)).*(w2.b(i) - w2.A(i,1)*x);
    plot(x,y,'b')
end
axis([-3000,3000,-3000,3000])
