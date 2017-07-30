%% Application for writing
fo = FileOperation(...
    'C:\Users\Tristan\Desktop\Constructing Demo\initstate.ini', ...
    'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\BrickArea_CurveThickWall.csv', ...
    'C:\Users\Tristan\Desktop\Constructing Demo\DATA FILES\CurveWallDesign20170721.csv');

time_step = 0.05;
% where the position of the EE with the acrylic board on the lower frame
% and pen cap on the pen.
q0 = [0 0 -0.2705 0 0 0]';

q_transit_point = [0 0 -0.2705 0 0 0]';

distance_safe = 0.05;
v_max = 0.120; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
blend_time_default = 0.5; %used to decide the acceleration
blend_time_placing = 1.5; %used to decide the deceleration

exp = PoCaBotExperiment(8,'dualcables_writing_UPDOWNSwitched');
exp.application_preparation(fo,q0);
q_temp = q0;
% lift the gripper from the ground
q_temp(3) = q0(3)+distance_safe;
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q_temp, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

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

k=3;
t = pi/7:0.05:pi*2-pi/7;
x = 0.05*cos(t);
x = [x 0 x(1)];
y = 0.05*sin(t) ;
y = [y 0 y(1)];
z = ones(size(x))* (-0.282);

% move to the position upright above the start point
q_uprightabove = [x(1) y(1) z(1)+0.02 0 0 0]';
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_uprightabove, time_step, 0.1, 0.1, v_max);
exp.runTrajectoryDirectly(trajectory);

for i=1:length(x)
    q = [x(i) y(i) z(i) 0 0 0]';
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q, time_step, 0.1, 0.1, v_max);
    exp.runTrajectoryDirectly(trajectory);
end

% move to the position upright above the end point
q1 = exp.q_present;
q1(3) = exp.q_present(3)+0.02;
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q1, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

t = 0:0.1:2*pi;
x = 0.004*cos(t);
y = 0.004*sin(t)+0.02;
z = ones(size(x))* (-0.282);
% move to the position upright above the end point
q1 = exp.q_present;
q1(3) = exp.q_present(3)+0.02;
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q1, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

% move to the position upright above the start point
q_uprightabove = [x(1) y(1) z(1)+0.02 0 0 0]';
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_uprightabove, time_step, 0.1, 0.1, v_max);
exp.runTrajectoryDirectly(trajectory);

for i=1:length(x)
    q = [x(i) y(i) z(i) 0 0 0]';
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q, time_step, 0.1, 0.1, v_max);
    exp.runTrajectoryDirectly(trajectory);
end

% move to the position upright above the end point
q1 = exp.q_present;
q1(3) = exp.q_present(3)+0.05;
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q1, time_step, blend_time_default, blend_time_default, v_max);
exp.runTrajectoryDirectly(trajectory);

exp.application_termination();
clear exp;
fprintf('Construction Finished.\n');