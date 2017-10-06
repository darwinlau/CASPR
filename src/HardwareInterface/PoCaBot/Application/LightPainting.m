%% Extract useful info from document

%% To read all value into the matrix 'values'
% MatrixToQ;
filename = 'unscale_new.csv';
delimiter = ',';
formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f\r';
fileID = fopen(which(filename),'r');
dataArray = textscan(fileID, formatSpec,'Delimiter', delimiter);
fclose(fileID);
tem = [dataArray{1:end}];
T = zeros(4,4,size(tem,1));
    for i = 1:size(tem,1)
       T(:,:,i) = [tem(i,(1:4));tem(i,(5:8));tem(i,(9:12));tem(i,(13:16))];
    end
clearvars filename delimiter formatSpec fileID dataArray tem i ans;

R = zeros(3,3,size(T,3));
    for i = 1:size(T,3)
       R(:,:,i) = T(1:3,1:3,i);
    end
clearvars i;

q = zeros(size(T,3),6);
    for i = 1:size(T,3)
        [a b g] = SphericalEulerXYZ.rotation_matrix_to_angles(R(:,:,i));
        q(i,:) = [(T(1,4,i)/1000),(T(2,4,i)/1000),(T(3,4,i)/1000),a ,b+1.57079632680000 ,g]; %with rotation
        %q(i,:) = [2,(T(2,4,i)/1000)+0.5,(T(3,4,i)/1000),0,0,0]; %no rotation
    end
clearvars i a b g;
q=q';
    


% Z = zeros(size(q,1),1);
%% Setup TCPIP feedback
 tcpipServer = tcpip('0.0.0.0',55000,'NetworkRole','Server');
 %fopen(tcpipServer);

%% Setup model 
fo = FileOperation(which('initstate.ini'));
distance_safe = 0.1;
time_step = 0.05;
% 
q0 = [1.377 1.795 0.772 0 0 0]';
%q0 = [2.0 2.0 0.27069 0 0 0]';
q_transit_point = [1.377 1.795 1 0 0 0]';
%q_transit_point = [2.0 2.0 1.0 0 0 0]';
v_op = 0.01; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
v_quick = 0.01;
blend_time_default = 0.5; %used to decide the acceleration
blend_time_placing = 1.5; %used to decide the deceleration

%% Preparing the robot from q0 to q1
if ~exist('exp','var')
    %exp = PoCaBotExperiment(8,'dualcables_lightpainting_endeffector_frame4by4by23_improvedjoint',time_step);   
    %exp = PoCaBotExperiment(8,'dualcables_lightpainting_new_endeffector_frame4by4by23_improvedjoint',time_step ,tcpipServer);
    
    exp = PoCaBotExperiment(8,'darwin_office',time_step ,tcpipServer);
    %exp = PoCaBotExperiment(8,'dualcables_small_endeffector_frame4by4by23_improvedjoint',time_step);   
    %exp = PoCaBotExperiment(8,'box_endeffector_frame4by4by23_improvedjoint_middle',time_step , tcpipServer);     
    exp.application_preparation(fo,q0);
    q_temp = q0;
    % lift the gripper to the preparation position
     q_temp(3) = q0(3)+distance_safe;
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q_transit_point, time_step, blend_time_default, blend_time_default, v_quick);
    exp.runTrajectoryDirectly(trajectory);
else
    q_temp = exp.q_present;
    q_temp(3) = q0(3)+distance_safe;
    % lift the gripper to the preparation position
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_transit_point, time_step, blend_time_default, blend_time_default, v_quick);
    exp.runTrajectoryDirectly(trajectory);
end

%% Move to the position upright above the start point
q_uprightabove = q(:,1);
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_uprightabove, time_step, 0.1, 0.1, 0.05);
exp.runTrajectoryDirectly(trajectory);

%% Run Trajectory
% count=2;
% tic;
% while (count <= size(q,2))
%     elapsed = toc - time_step;
%     if(elapsed>0)
% exp.model.update(q(:,count), Z, Z,Z);
% % [~, model_temp, ~, ~, ~] = exp.idsim.IDSolver.resolve(q, Z, Z, Z);
% % [offset] = exp.hardwareInterface.getCableOffsetByTensionByMotorAngleError(model_temp.cableForces);
% % exp.hardwareInterface.lengthCommandSend(model_temp.cableLengths ./(1+exp.factor_offset_per_Newton_Meter*model_temp.cableForces) + offset);
%     exp.hardwareInterface.lengthCommandSend(exp.model.cableLengths);
%     tic;
%     count = count + 1;
%     end
% %     exp.l_feedback_traj(:, i) = exp.hardwareInterface.lengthFeedbackRead;
% end
for i=1:size(q, 2)
        trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q(:,i), time_step, 0, 0, 0.1);
        
%         trajectory.q = trajectory.q(:, 1:(size(trajectory.q,2)-1));
%         trajectory.q_dot = trajectory.q_dot(:, 1:(size(trajectory.q_dot,2)-1));
%         trajectory.q_ddot = trajectory.q_ddot(:, 1:(size(trajectory.q_ddot,2)-1));
%         trajectory.timeVector = trajectory.timeVector(:, 1:(size(trajectory.timeVector,2)-1));
        exp.runTrajectoryDirectly(trajectory);
        fprintf('                              Current progress %.2f %%\n', i/size(q, 2)*100) % Print current progress
end
clear i

%% Move back to initial point and terminate
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_transit_point, time_step, 0.1, 0.1, v_quick);
exp.runTrajectoryDirectly(trajectory);
%exp.application_termination();
%clear;
%fclose(tcpipServer);
fprintf('Have a nice photo:)\n');
