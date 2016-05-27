model_config = ModelConfig(ModelConfigType.M_IPANEMA_2);
trajectory_id = 'traj_z_up';
cable_set_id = 'default';

% The XML objects from the model config are created
bodies_xmlobj = model_config.getBodiesPropertiesXmlObj();
cableset_xmlobj = model_config.getCableSetXmlObj(cable_set_id);
trajectory_xmlobj = model_config.getTrajectoryXmlObj(trajectory_id);

% Load the SystemKinematics object from the XML
kinObj = SystemModel.LoadXmlObj(bodies_xmlobj, cableset_xmlobj);

id_objective = IDObjectiveMinQuadCableForce(ones(kinObj.numCables,1));
idsolver = IDSolverQuadProg(kinObj, id_objective, ID_QP_SolverType.MATLAB);
trajectory = JointTrajectory.LoadXmlObj(trajectory_xmlobj, kinObj);

numCables = 8;
ap_cables = cell(numCables, 1);

for i = 1:numCables
    ap_const = AttachmentPointParamConstant(kinObj.cableModel.cables{i}.segments{1}.r_PA{2}, CableAttachmentReferenceType.JOINT);
    ap_rad = AttachmentPointParamCylindricalFixedR(4, [0 2*pi], [0 5], CableAttachmentReferenceType.JOINT);
    ap_cables{i} = AttachmentPointParamCable({ap_rad, ap_const});
end

ap_system = AttachmentPointParamSystem(ap_cables);

cableAttachmentOptimisation = CableAttachmentOptimisationMinID(kinObj, ap_system, 1e7);
optimiser = PSOOptimiser(ap_system.x_min, ap_system.x_max, @(x) cableAttachmentOptimisation.evaluate(x, idsolver, trajectory));
[x_opt, Q_opt] = optimiser.optimise();

ap_system.updateCableAttachments(x_opt, kinObj.cableModel, kinObj.bodyModel);
kinObj.update(zeros(kinObj.numDofs,1), zeros(kinObj.numDofs,1), zeros(kinObj.numDofs,1), zeros(kinObj.numDofs,1));
MotionSimulator.PlotFrame(kinObj, [-3 3 -3 3 0 5]);


% Setup the inverse dynamics simulator with the SystemKinematicsDynamics
% object and the inverse dynamics solver
disp('Start Setup Simulation');
start_tic = tic;
idsim = InverseDynamicsSimulator(kinObj, idsolver);
time_elapsed = toc(start_tic);
fprintf('End Setup Simulation : %f seconds\n', time_elapsed);

% Run the solver on the desired trajectory
disp('Start Running Simulation');
start_tic = tic;
idsim.run(trajectory);
time_elapsed = toc(start_tic);
fprintf('End Running Simulation : %f seconds\n', time_elapsed);
idsim.plotCableForces([], []);

