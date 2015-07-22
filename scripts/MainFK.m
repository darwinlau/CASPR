% Load configs
clc; clear; close all;

% Planar model
model_config = ModelConfig(ModelConfigType.M_PLANAR_XY);
trajectory_id = 'x_simple';
cable_set_id = 'basic';

% % 8S neck model
% model_config = ModelConfig(ModelConfigType.M_NECK_8S);
% trajectory_id = 'roll';
% cable_set_id = 'opensim_vasavada';

bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);

disp('Start Setup Simulation');
start_tic = tic;

kinObj = SystemKinematics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);
iksim = InverseKinematicsSimulator(kinObj);
fksim = ForwardKinematicsSimulator(kinObj);
iksim2 = InverseKinematicsSimulator(kinObj);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, kinObj);

time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

disp('Start Running Inverse Kinematics Simulation');
start_tic = tic;

iksim.run(trajectory);

time_elapsed = toc(start_tic);
fprintf('End Running Inverse Kinematics Simulation : %f seconds\n', time_elapsed);

disp('Start Running Forward Kinematics Simulation');
start_tic = tic;

fksim.run(iksim.lengths, iksim.lengths_dot, iksim.timeVector, iksim.trajectory.q{1}, iksim.trajectory.q_dot{1});

time_elapsed = toc(start_tic);
fprintf('End Running Forward Kinematics Simulation : %f seconds\n', time_elapsed);

% disp('Start Running Inverse Kinematics 2 Simulation');
% start_tic = tic;
% iksim2.run(fksim.trajectory);
% time_elapsed = toc(start_tic);
% fprintf('End Running Inverse Kinematics 2 Simulation : %f seconds\n', time_elapsed);

iksim.plotJointSpace();
fksim.plotJointSpace();
%iksim2.plotJointSpace();

for i = 1:length(iksim.timeVector)
    q_error(i) = norm(iksim.trajectory.q{i} - fksim.trajectory.q{i});
end
plot(iksim.timeVector, q_error);

