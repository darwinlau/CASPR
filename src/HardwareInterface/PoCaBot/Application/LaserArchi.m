%% Extract useful info from csv
[filename, pathname] = uigetfile('*.csv;','Pick a CSV path');
fileID = fopen(strcat(pathname,filename),'r');
delimiter = ',';
formatSpec = '%f%f%f%f%[^\n\r]';
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);
fclose(fileID);
x = dataArray{1:1};
y = dataArray{2:2};
z = ones(size(x))* 0.23;
clearvars filename delimiter formatSpec fileID ans pathname dataArray;

%% Laser end-effector setup
if ~exist('gripper','var')
    gripper = Gripper('COM13');
    gripper.initialize();
end

gripper.setLaser(1);
time_step = 0.05;

%% Setup model 
fo = FileOperation(which('initstate.ini'));
q0 = [2.0 2.0 0.19 0 0 0]';
q_transit_point = [2.0 2.0 1.0 0 0 0]';
distance_safe = 0.1;
v_max = 0.02; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
blend_time_default = 0.001; %used to decide the acceleration
blend_time_placing = 1.5;

%% Preparing the robot from q0 to q1
if ~exist('exp','var')
    exp = PoCaBotExperiment(8,'dualcables_large_endeffector_frame4by4by23_demo',time_step);
    exp.application_preparation(fo, q0);
    q_temp = q0;
    % lift the gripper from the ground
    q_temp(3) = q0(3)+distance_safe;
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q_temp, time_step, 0.5, 0.5, 0.1);
    exp.runTrajectoryDirectly(trajectory);
else
    q_temp = exp.q_present;
    q_temp(3) = q0(3)+distance_safe;
    % lift the gripper to prevent collision
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_temp, time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);
end

%% Move to the position upright above the start point
q_uprightabove = [x(1) y(1) z(1)+0.02 0 0 0]';
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_uprightabove, time_step, 0.1, 0.1, 0.1);
exp.runTrajectoryDirectly(trajectory);
gripper.setLaser(2);

%% Run Trajectory
for i=1:length(x)-1
    q = [x(i) y(i) z(i) 0 0 0]';
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q, time_step, 0.1, 0.1, v_max);
    exp.runTrajectoryDirectly(trajectory);
end

gripper.setLaser(1);

%% Move back to initial point and terminate
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_transit_point, time_step, 0.1, 0.1, 0.1);
exp.runTrajectoryDirectly(trajectory);
%exp.application_termination();
%clear;
%fprintf('Laser Cutting Done:)\n');