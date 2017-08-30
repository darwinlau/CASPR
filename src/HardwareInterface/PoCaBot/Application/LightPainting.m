% fo = FileOperation(...
%     'C:\Users\Tristan\Desktop\Constructing Demo\initstate.ini', ...
%     'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\Wall4_BrickArea0731.csv', ...
%     'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\Wall4_Design0731.csv');

time_step = 1.0;

q0 = [2.0 2.0 0.424 0 0 0]';
q_transit_point = [2.0 2.0 1.0 0 0 0]';

v_max = 0.120; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
blend_time_default = 0.5; %used to decide the acceleration
blend_time_placing = 1.5; %used to decide the deceleration

if ~exist('exp','var')
    exp = PoCaBotExperiment(8,'dualcables_large_endeffector_frame4by4by23_demo',time_step);
    exp.application_preparation(fo,q0);

    % lift the gripper to the preparation position
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q_transit_point, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
else
    q_temp = exp.q_present;
    % lift the gripper to the preparation position
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_transit_point, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
end

% The below program is just for debugging. When working for the task, please
% comment these expressions.
% while(1)
%     factor = input('The offset constant factor[Nothing means no changing!]:');
%     if ~isempty(factor)
%         exp.factor_offset_per_Newton_Meter = factor;
%     end
%     fprintf('The factor is %0.5f from now on!\n',exp.factor_offset_per_Newton_Meter);
%     q_next = (input('The next q:'))';
%     trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_next, time_step, blend_time_placing, blend_time_placing, v_max);
%     exp.runTrajectoryDirectly(trajectory);
% end

MatrixToQ;
Z = zeros(size(q,1),1);
count=1;
tic;
while (count <= size(q,2))
    elapsed = toc - timestep;
    if(elapsed>0)
    exp.model.update(q(:,count), Z, Z);
    hardwareInterface.lengthCommandSend(exp.model.cableLengths);
    tic;
    count = count + 1;
    end
%     exp.l_feedback_traj(:, i) = exp.hardwareInterface.lengthFeedbackRead;
end

%exp.application_termination();
%clear exp;
fprintf('Exposure Finished.\n');


