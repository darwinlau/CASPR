% Script file to test the use of foward kinematics on a setup where only
% relative lengths of the cables are available
%
% Author        : Darwin LAU
% Created       : 2015
% Description    :

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
% Following are some examples (feel free to add more):
% 1) Planar model
model_config = ModelConfig(ModelConfigType.M_SIMPLE_PLANAR_XY);
trajectory_id = 'general_2';
cable_set_id = 'basic';
q0_error = [0.05; 0.05; 2*pi/180];
% 2) Neck model
% model_config = ModelConfig(ModelConfigType.M_NECK_8S);
% trajectory_id = 'roll';
% cable_set_id = 'opensim_vasavada';
% 3) TUM Myorob arm model
% model_config = ModelConfig(ModelConfigType.M_MYOROB_SHOULDER);
% trajectory_id = 'traj_1';
% cable_set_id = 'myorob_shoulder';
% q0_error = [20*pi/180; -30*pi/180; 15*pi/180];

% The XML objects from the model config are created
bodies_xmlobj = model_config.getBodiesProperiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);

% Load the SystemKinematics object from the XML
kinObj = SystemKinematics.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

% Three simulators will be setup for this work to show the forward
% kinematics working:
% 1) iksim_true: InverseKinematicsSimulator holding the true trajectory
% 2) fksim_error: ForwardKinematicsSimulator using erroneous initial state
% 3) fksim_corrected: ForwardKinematicsSimulator using corrected init state
disp('Start Setup Simulation');
start_tic = tic;
% Initialise the least squares solver for the forward kinematics
fksolver = FKLeastSquares(kinObj, FK_LS_ApproxOptionType.FIRST_ORDER_INTEGRATE_PSEUDOINV, FK_LS_QdotOptionType.PSEUDO_INV);
% Initialise the three inverse/forward kinematics solvers
iksim_true = InverseKinematicsSimulator(kinObj);
fksim_error = ForwardKinematicsSimulator(kinObj, fksolver);
fksim_corrected = ForwardKinematicsSimulator(kinObj, fksolver);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, kinObj);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% The three simulators will be run in the following way:
% 1. The iksim will be run with the correct trajectory to obtain the
% correct cable lengths
% 2. An erroneous initial cable lengths will be used to determine an
% erroneous cable length profile using the erroneous initial length but
% correct relative lengths from step 1
% 3. The forward kinematics simulator will be used on the erroneous cable
% lengths to determine the joint trajectory profile
% 4. Using the relative lengths and erroneous joint space trajectory
% obtained from 3, the FK solver will try to solve for the correct initial
% lengths (l0_solved)
% 5. Re-determine the cable length profile using the initial lengths from
% step 4.
% 6. The forward kinematics simulator will be used on the corrected cable
% lengths from step 5.

% Step 1
disp('Start Running Inverse Kinematics Simulation');
start_tic = tic;
iksim_true.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Inverse Kinematics Simulation : %f seconds\n', time_elapsed);

% step 2
% Compute the deviated lengths trajectory if inaccurate q0 and l0 are used
q0 = iksim_true.trajectory.q{1};
l0 = iksim_true.lengths{1};
% Apply some deviation to q0
q0_dev = q0 + q0_error;
kinObj.update(q0_dev, zeros(size(q0_dev)), zeros(size(q0_dev)));
% Obtain the deviated l0
l0_dev = kinObj.cableLengths;
lengths_r = cell(size(iksim_true.lengths));
lengths_dev = cell(size(iksim_true.lengths));
for t = 1:length(iksim_true.lengths)
    % Determine the relative lengths (what the sensor should read)
    %   Sensor noise can be included here
    lengths_r{t} = iksim_true.lengths{t} - l0;% + 0.001*randn(kinObj.numCables, 1);
    % Determine the deviated lengths if an inaccurate l0 is used
    lengths_dev{t} = lengths_r{t} + l0_dev;
end

% Step 3
disp('Start Running Forward Kinematics Simulation for Deviated Lengths');
start_tic = tic;
fksim_error.run(lengths_dev, iksim_true.lengths_dot, iksim_true.timeVector, q0_dev, iksim_true.trajectory.q_dot{1});
time_elapsed = toc(start_tic);
fprintf('End Running Forward Kinematics Simulation for Deviated Lengths : %f seconds\n', time_elapsed);

% Step 4
disp('Start Running Solver for Initial Lengths');
start_tic = tic;
l0_solved = FKLeastSquares.ComputeInitialLengths(kinObj, lengths_r, l0_dev, fksim_error.trajectory.q);
time_elapsed = toc(start_tic);
fprintf('End Running Solver for Initial Lengths : %f seconds\n', time_elapsed);

% Step 5
lengths_solved = cell(size(iksim_true.lengths{t}));
for t = 1:length(iksim_true.lengths)
    lengths_solved{t} = lengths_r{t} + l0_solved;
end

% Step 6
disp('Start Running Forward Kinematics Simulation for Solved l0 Lengths');
start_tic = tic;
fksim_corrected.run(lengths_solved, iksim_true.lengths_dot, iksim_true.timeVector, q0_dev, iksim_true.trajectory.q_dot{1});
time_elapsed = toc(start_tic);
fprintf('End Running Forward Kinematics Simulation for Solved l0 Lengths : %f seconds\n', time_elapsed);

% Display the results
fprintf('The true initial lengths:\n');
disp(l0);
fprintf('The incorrect initial lengths:\n');
disp(l0_dev);
fprintf('The solved initial lengths:\n');
disp(l0_solved);

% It is expected that iksim_true and fksim_corrected should be similiar
iksim_true.plotJointSpace();
fksim_error.plotJointSpace();
fksim_corrected.plotJointSpace();