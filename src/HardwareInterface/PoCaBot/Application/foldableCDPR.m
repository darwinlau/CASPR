%% Application for Foldable CDPR
if(~exist('exp','var'))
    % Initial length calibration
    exp = PoCaBotExperiment(8,'dualcables_foldable_writing_UPDOWNSwitched');
    % Some random initial length guess
    l0_guess = ones(8,1)*0.3;
    duration = 40;% seconds
    [q_initial] = exp.initialLenQCalibration(l0_guess,duration);
end


time_step = 0.05;
v_max = 0.120; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
blend_time_default = 0.1;

% while(1)
%     q_next = (input('The next q:'))';
%     trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_next, time_step, blend_time_default, blend_time_default, v_max);
%     exp.runTrajectoryDirectly(trajectory);
% end

% Go back to the initial position
fprintf('Now go back to the initial position.\n');
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_initial, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

% Run the trajectory.
angle = pi*2;
t = [0:0.1:angle]';
x = 0.1*cos(t)+0.27;
y = 0.1*sin(t)+0.27;
z = t/angle*0.1+0.25;

for i = 1:numel(t)
    q_next = [x(i) y(i) z(i) 0 0 0]';
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_next, time_step, blend_time_default, blend_time_default, v_max);
    exp.runTrajectoryDirectly(trajectory);
    %PoCaBotExperiment.plotTrajectory(trajectory);
end

% Go back to the centric position
fprintf('Now go back to the centric position.\n');
q_centric = [0.27 0.27 0.30 0 0 0]';
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_centric, time_step, blend_time_default, blend_time_default, v_max/2);
exp.runTrajectoryDirectly(trajectory);

% Do roll pitch YAW rotation
fprintf('Now go back to the centric position.\n');
q_centric = [0.27 0.27 0.30 0 0 0.4]';
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_centric, time_step, blend_time_default, blend_time_default, v_max/2);
exp.runTrajectoryDirectly(trajectory);
q_centric = [0.27 0.27 0.30 0 0 -0.4]';
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_centric, time_step, blend_time_default, blend_time_default, v_max/2);
exp.runTrajectoryDirectly(trajectory);
q_centric = [0.27 0.27 0.30 0 0 0]';
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_centric, time_step, blend_time_default, blend_time_default, v_max/2);
exp.runTrajectoryDirectly(trajectory);

% Go back to the initial position
fprintf('Now go back to the initial position.\n');
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_initial, time_step, blend_time_default, blend_time_default, v_max/2);
exp.runTrajectoryDirectly(trajectory);

%exp.application_termination();
%clear exp;
fprintf('Mission Finished.\n');
