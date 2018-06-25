clc; clear; close all;

model_config = ModelConfig('IPAnema 2');
trajectory_id = 'traj_z_up';
cable_set_id = 'original';

modelObj = model_config.getModel(cable_set_id);
id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numCables,1));
idsolver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);
trajectory = model_config.getJointTrajectory(trajectory_id);

numCables = 8;

ap_cables = cell(numCables, 1);

for i = 1:numCables
    ap_rad = AttachmentPointParamCylindricalFixedR(modelObj.cableModel.cables{i}.attachments{1}, CableAttachmentReferenceType.JOINT, 4, [0 2*pi], [0 5], [0;0;0], [0;0;1]);    
    %ap_const = AttachmentPointParamConstant(modelObj.cableModel.cables{i}.attachments{2});
    ap_cables{i} = AttachmentPointParamCable({ap_rad});
end

ap_system = AttachmentPointParamSystem(ap_cables);

cableAttachmentOptimisation = CableAttachmentOptimisationMinID(modelObj, ap_system, 1e7);
optimiser = PSOOptimiser(ap_system.x_min, ap_system.x_max, @(x) cableAttachmentOptimisation.evaluate(x, idsolver, trajectory));
[x_opt, Q_opt] = optimiser.optimise();

ap_system.updateCableAttachments(x_opt);
modelObj.update(zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1), zeros(modelObj.numDofs,1));
MotionSimulatorBase.PlotFrame(modelObj, [-3 3 -3 3 0 5], [-37, 32]);


% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
idsim = InverseDynamicsSimulator(modelObj, idsolver);

% Run the solver on the desired trajectory
disp('Start Running Simulation');
idsim.run(trajectory);
idsim.plotCableForces();

