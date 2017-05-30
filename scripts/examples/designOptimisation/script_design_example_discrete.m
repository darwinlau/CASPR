clc; clear; close all;

% Pick a suitable robot
% 

% Start by just confirming that it works

model_config = DevModelConfig(DevModelConfigType.D_CUHK_TEMP);
% cable_set_id_4 = 'basic_4_cables';
% 
% modelObj_4 = model_config.getModel(cable_set_id_4);
% % id_objective = IDObjectiveMinQuadCableForce(ones(modelObj.numCables,1));
% % idsolver = IDSolverQuadProg(modelObj, id_objective, ID_QP_SolverType.MATLAB);
% % trajectory = model_config.getJointTrajectory(trajectory_id);
% 
% q_step          =   pi/8; n_dim           =   3;
% uGrid           =   UniformGrid([-pi/2;-pi/2;-pi],[pi/2;pi/2;pi],q_step*ones(n_dim,1));
% 
% 
% numCables = 4;
% ap_cables = cell(numCables, 1);
% discrete_lists = cell(4*numCables,1);
% for i = 1:numCables
%     % Needs to be changed
%     ap_rad = AttachmentPointParamCylindricalFixedR(0.025, [0 2*pi], [0.03 0.17], [0;0;0],[0;-1;0], CableAttachmentReferenceType.JOINT);
%     %ap_const = AttachmentPointParamConstant(modelObj.cableModel.cables{i}.segments{1}.r_PA{2}, CableAttachmentReferenceType.JOINT);
%     ap_pol = AttachmentPointParamPolarPlanar([0.1 0.4], [0 2*pi], 0, [0;0;0],[0;-1;0], CableAttachmentReferenceType.JOINT);
%     ap_cables{i} = AttachmentPointParamCable({ap_pol, ap_rad});
%     discrete_lists{4*i-3} = 0.1:0.05:0.4;
%     discrete_lists{4*i-2} = 0:pi/8:2*pi;
%     discrete_lists{4*i-1} = 0:pi/4:2*pi;
%     discrete_lists{4*i} = 0.03:0.02:0.17;
% end
% 
% ap_system = AttachmentPointParamSystem(ap_cables);
% 
% cableAttachmentOptimisation = CableAttachmentOptimisationSEACM(modelObj_4, ap_system, 1e7);
% optimiser = PSOOptimiserDiscrete(ap_system.x_min, ap_system.x_max, @(x) cableAttachmentOptimisation.evaluate(x, uGrid),1,discrete_lists);
% [x_opt, Q_opt] = optimiser.optimise();
% 
% ap_system.updateCableAttachments(x_opt, modelObj_4.cableModel, modelObj_4.bodyModel);
% modelObj_4.update(zeros(modelObj_4.numDofs,1), zeros(modelObj_4.numDofs,1), zeros(modelObj_4.numDofs,1), zeros(modelObj_4.numDofs,1));
% MotionSimulator.PlotFrame(modelObj_4, [-1 1 -1 1 -1 1], [-37,32]);

% Same problem 6 cables
cable_set_id_6 = 'basic_6_cables';

modelObj_6 = model_config.getModel(cable_set_id_6);

q_step          =   pi/9; n_dim           =   3;
uGrid           =   UniformGrid([-pi/3;-pi/3;-pi],[pi/3;pi/3;pi],q_step*ones(n_dim,1));


numCables = 6;
ap_cables = cell(numCables, 1);
discrete_lists = cell(4*numCables,1);
for i = 1:numCables
    % Needs to be changed
    ap_rad = AttachmentPointParamCylindricalFixedR(0.025, [0 2*pi], [0.05 0.17], [0;0;0],[0;-1;0], CableAttachmentReferenceType.JOINT);
    %ap_const = AttachmentPointParamConstant(modelObj.cableModel.cables{i}.segments{1}.r_PA{2}, CableAttachmentReferenceType.JOINT);
    ap_pol = AttachmentPointParamPolarPlanar([0.1 0.2], [0 2*pi], 0, [0;0;0],[0;-1;0], CableAttachmentReferenceType.JOINT);
    ap_cables{i} = AttachmentPointParamCable({ap_pol, ap_rad});
    discrete_lists{4*i-3} = 0.1:0.05:0.2;
    discrete_lists{4*i-2} = 0:pi/8:2*pi;
    discrete_lists{4*i-1} = 0:pi/2:2*pi;
    discrete_lists{4*i} = 0.05:0.02:0.17;
end

ap_system = AttachmentPointParamSystem(ap_cables);

cableAttachmentOptimisation = CableAttachmentOptimisationSEACM(modelObj_6, ap_system, 1e7);
optimiser = PSOOptimiserDiscrete(ap_system.x_min, ap_system.x_max, @(x) cableAttachmentOptimisation.evaluate(x, uGrid),1,discrete_lists);
[x_opt, Q_opt] = optimiser.optimise();

ap_system.updateCableAttachments(x_opt, modelObj_6.cableModel, modelObj_6.bodyModel);
modelObj_6.update(zeros(modelObj_6.numDofs,1), zeros(modelObj_6.numDofs,1), zeros(modelObj_6.numDofs,1), zeros(modelObj_6.numDofs,1));
MotionSimulator.PlotFrame(modelObj_6, [-1 1 -1 1 -1 1], [-37,32]);
