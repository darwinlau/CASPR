% Script file to show how to use the inverse dynamics simulator
% 
% Author        : Darwin LAU
% Created       : 2012
% Description	:
% Known issues  : 1) Possible issues with the ODE solvers for complex
%                    problems

% Clear the variables, command window, and all windows
clc; clear; close all;

% Set up the type of model, trajectory and the set of cables to be used
% Following are some examples (feel free to add more):
% model_config = ModelConfig(ModelConfigType.M_SIMPLE_PLANAR_XY);
% cable_set_id = 'basic';
% trajectory_id = 'x_simple';

% modelObj = model_config.getModel(cable_set_id);
model_config = DevModelConfig(DevModelConfigType.D_CUHK_ARM);
trajectory_id = 'traj_test_ol_';
modelObj = model_config.getModel(model_config.defaultCableSetId);

% Setup an inverse dynamics solver of choice (any should do)
id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numCables,1));
id_solver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);

% Setup the inverse dynamics and forward dynamics simulators
disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(modelObj, id_solver);
fdsim = ForwardDynamicsSimulator(modelObj, FDSolverType.ODE113);
trajectory = model_config.getJointTrajectory(trajectory_id);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% First run the inverse dynamics
disp('Start Running Inverse Dynamics Simulation');
start_tic = tic;
idsim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Inverse Dynamics Simulation : %f seconds\n', time_elapsed);

% idsim.plotJointSpace();
% drawnow;

% fdsim.run(idsim.cableForcesActive, idsim.cableIndicesActive, trajectory.timeVector, trajectory.q{1}, trajectory.q_dot{1});
% fdsim.plotJointSpace();

H = eye(modelObj.numCables); g = zeros(modelObj.numCables,1);
forces_active_new = cell(size(idsim.cableForcesActive));
true_q = trajectory.q{1}; true_qd = trajectory.q_dot{1};
f = figure; a = gca; hold on;
for i =1:length(trajectory.timeVector)-1
    i
    modelObj.update(true_q,true_qd,trajectory.q_ddot{i},zeros(modelObj.numDofs,1));
    A_i = -modelObj.L';
    b_i = modelObj.M*(trajectory.q_ddot{i}+diag([5,5,5,5])*(trajectory.q{i}-true_q) + diag([0.5,0.5,0.5,0.5])*(trajectory.q_dot{i}-true_qd)) + modelObj.C + modelObj.G;
    forces_active_new{i} = quadprog(H,g,[],[],A_i,b_i,0*ones(modelObj.numCables,1),1000*ones(modelObj.numCables,1));
    fdsim.run(forces_active_new(i:i+1), idsim.cableIndicesActive(i:i+1), trajectory.timeVector(i:i+1), true_q, true_qd);
    true_q  =   fdsim.trajectory.q{2};
    true_qd =   fdsim.trajectory.q_dot{2};
    fdsim.plotJointSpace(a);
%     drawnow;
end

% An alternative to the fd
H = eye(modelObj.numCables); g = zeros(modelObj.numCables,1);
forces_active_new = cell(size(idsim.cableForcesActive));
true_q = trajectory.q{1}; true_qd = trajectory.q_dot{1};
f = figure; a = gca; hold on;
for i =1:length(trajectory.timeVector)-1
    i
    modelObj.update(true_q,true_qd,trajectory.q_ddot{i},zeros(modelObj.numDofs,1));
    A_i = -modelObj.L';
    b_i = modelObj.M*(trajectory.q_ddot{i}+diag([5,5,5,5])*(trajectory.q{i}-true_q) + diag([0.5,0.5,0.5,0.5])*(trajectory.q_dot{i}-true_qd)) + modelObj.C + modelObj.G;
    modelObj.update(trajectory.q{i+1},trajectory.q_dot{i+1},trajectory.q_ddot{i+1},zeros(modelObj.numDofs,1));
    A_ip1 = -modelObj.L';
    b_ip1 = modelObj.M*(trajectory.q_ddot{i+1}+diag([5,5,5,5])*(trajectory.q{i}-true_q) + diag([0.5,0.5,0.5,0.5])*(trajectory.q_dot{i}-true_qd)) + modelObj.C + modelObj.G;
%     forces_active_new{i} = quadprog(H,g,[],[],A_i,0.5*(b_i+b_ip1),0*ones(modelObj.numCables,1),1000*ones(modelObj.numCables,1));
    forces_active_new{i} = quadprog(H,g,[],[],0.5*(A_i+A_ip1),0.5*(b_i+b_ip1),0*ones(modelObj.numCables,1),1000*ones(modelObj.numCables,1));
    test = forces_active_new{i}
    fdsim.run(forces_active_new(i:i+1), idsim.cableIndicesActive(i:i+1), trajectory.timeVector(i:i+1), true_q, true_qd);
    true_q  =   fdsim.trajectory.q{2};
    true_qd =   fdsim.trajectory.q_dot{2};
    fdsim.plotJointSpace(a);
%     drawnow;
end
% fdsim.run(forces_active_new(1:100), idsim.cableIndicesActive(1:100), trajectory.timeVector(1:100), trajectory.q{1}, trajectory.q_dot{1});
% fdsim.plotJointSpace();

% 
% H = eye(modelObj.numCables); g = zeros(modelObj.numCables,1);
% forces_active_new = cell(size(idsim.cableForcesActive));
% h = trajectory.timeVector(2) - trajectory.timeVector(1);
% options = optimset('Algorithm','active-set');
% for i =1:100
%     i
%     modelObj.update(trajectory.q{i},trajectory.q_dot{i},trajectory.q_ddot{i},zeros(modelObj.numDofs,1));
%     A_i = -modelObj.L';
%     b_i = modelObj.M*trajectory.q_ddot{i} + modelObj.C + modelObj.G;
%     modelObj.update(trajectory.q{i+1},trajectory.q_dot{i+1},trajectory.q_ddot{i+1},zeros(modelObj.numDofs,1));
%     A_ip1 = -modelObj.L';
%     b_ip1 = modelObj.M*trajectory.q_ddot{i+1} + modelObj.C + modelObj.G;
%     A = (((1/3)-(1/3)*(1-h)^3)*A_i'*A_i + (0.5*h^2 - (1/3)*h^3)*(A_i'*A_ip1 + A_ip1'*A_i) + (1/3)*h^3*A_ip1'*A_ip1);    
%     b = (((1/3)-(1/3)*(1-h)^3)*b_i'*A_i + (0.5*h^2 - (1/3)*h^3)*(b_i'*A_ip1 + b_ip1'*A_i) + (1/3)*h^3*b_ip1'*A_ip1);
%     forces_active_new{i} = quadprog(H+1e10*A,g-1e10*b',[],[],[],[],0*ones(modelObj.numCables,1),1000*ones(modelObj.numCables,1),[],options);
% end
% fdsim.run(forces_active_new(1:100), idsim.cableIndicesActive(1:100), trajectory.timeVector(1:100), trajectory.q{1}, trajectory.q_dot{1});
% fdsim.plotJointSpace();
    

% H = eye(modelObj.numCables); g = zeros(modelObj.numCables,1);
% forces_active_new = cell(size(idsim.cableForcesActive));
% h = trajectory.timeVector(2) - trajectory.timeVector(1);
% for i =1:length(trajectory.timeVector)-1
%     i
%     modelObj.update(trajectory.q{i},trajectory.q_dot{i},trajectory.q_ddot{i},zeros(modelObj.numDofs,1));
%     A_i = -modelObj.L';
%     b_i = modelObj.M*trajectory.q_ddot{i} + modelObj.C + modelObj.G;
%     modelObj.update(trajectory.q{i+1},trajectory.q_dot{i+1},trajectory.q_ddot{i+1},zeros(modelObj.numDofs,1));
%     A_ip1 = -modelObj.L';
%     b_ip1 = modelObj.M*trajectory.q_ddot{i+1} + modelObj.C + modelObj.G;
%     A = ((h-0.5*h^2)^2*A_i'*A_i + 0.5*h^2*(h-0.5*h^2)*(A_i'*A_ip1 + A_ip1'*A_i) + 0.25*h^4*A_ip1'*A_ip1);    
%     b = ((h-0.5*h^2)^2*b_i'*A_i + 0.5*h^2*(h-0.5*h^2)*(b_i'*A_ip1 + b_ip1'*A_i) + 0.25*h^4*b_ip1'*A_ip1);    
%     forces_active_new{i} = quadprog(H,g,[],[],1000*A,1000*b',0*ones(modelObj.numCables,1),1000*ones(modelObj.numCables,1));
% end
% fdsim.run(forces_active_new, idsim.cableIndicesActive, trajectory.timeVector, trajectory.q{1}, trajectory.q_dot{1});
% fdsim.plotJointSpace();
    

% 
% f = figure; a = gca; hold on;
% f1 = figure; a1 = gca; hold on;
% % Then run the forward dynamics
% disp('Start Running Forward Dynamics Simulation');
% start_tic = tic;
% H = eye(modelObj.numCables); g = zeros(modelObj.numCables,1);
% for i =1:length(trajectory.timeVector)-1
%     fdsim.run(idsim.cableForcesActive(i:i+1), idsim.cableIndicesActive(i:i+1), trajectory.timeVector(i:i+1), trajectory.q{i}, trajectory.q_dot{i});
% %     fdsim.plotJointSpace(a);
% %     drawnow;
%     % Compute the alternative
%     % Take the averages
%     modelObj.update(trajectory.q{i},trajectory.q_dot{i},trajectory.q_ddot{i},zeros(modelObj.numDofs,1));
%     A_i = -modelObj.L';
%     b_i = modelObj.M*trajectory.q_ddot{i} + modelObj.C + modelObj.G;
%     modelObj.update(trajectory.q{i+1},trajectory.q_dot{i+1},trajectory.q_ddot{i+1},zeros(modelObj.numDofs,1));
%     A_ip1 = -modelObj.L';
%     b_ip1 = modelObj.M*trajectory.q_ddot{i+1} + modelObj.C + modelObj.G;
%     forces_active = quadprog(H,g,[],[],0.5*(A_i+A_ip1),0.5*(b_i+b_ip1),0*ones(modelObj.numCables,1),1000*ones(modelObj.numCables,1));
%     fdsim.run(mat2cell(forces_active,modelObj.numCables,1), idsim.cableIndicesActive(i:i+1), trajectory.timeVector(i:i+1), trajectory.q{i}, trajectory.q_dot{i});
% %     fdsim.plotJointSpace(a1);
% %     drawnow;
% end
time_elapsed = toc(start_tic);
fprintf('End Running Forward Dynamics Simulation : %f seconds\n', time_elapsed);
