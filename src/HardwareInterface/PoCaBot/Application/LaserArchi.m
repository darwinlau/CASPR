%% Define workspace properties
fitToWorkspace = 1;  %enable this to fit the pattern to the workspace

%frameSize = [0.8430 0.54 0.8]';   %small laserbot
frameSize = [2.7 3.3 2.3];
%cuboidSize = [0.4 0.4 0.66]'; %small laserbot
cuboidSize = [2.5 2.5 1.8]';
    if fitToWorkspace
        shrinkWorkspace = 0.4;  %decide how close the pattern is to the margin of the workspace
    else
        shrinkWorkspace = 1;
    end
workspaceSize = cuboidSize.*shrinkWorkspace;
workspaceCenter = frameSize./2; 




%% Extract useful info from csv
%For Archi Csv files
%uigetfile({'*.csv;';'*.gcode'},{'CSV file (*.csv)'},'Pick a CSV path');
[filename, pathname] = uigetfile({'*.csv;','CSV file (*.csv)';'*.gco;*.gcode;*.g;','Gcode Files (*.gco,*.gcode,*.g)'; },'Pick a file');

fileID = fopen(strcat(pathname,filename),'r');
[~,~,ext] = fileparts(filename);

if strcmp(ext,'.csv')
    delimiter = ',';
    formatSpec = '%f%f%f%f%[^\n\r]';
    dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);
    fclose(fileID);
    x = dataArray{1:1}/1000;
    y = dataArray{2:2}/1000;
    e = ones(1,size(x,1));
    
elseif strcmp(ext,'.gco') || strcmp(ext,'.gcode') || strcmp(ext,'.g')
    %For our gcode
%     [filename, pathname] = uigetfile({'*.gco;*.gcode;*.g;','Gcode Files (*.gco,*.gcode,*.g)'},'Pick a file');
%     fileID = fopen(strcat(pathname,filename),'r');
    gcode = textscan(fileID, '%s','Delimiter', '\r');
    gcode = gcode{:,1};
    Zflag = 0;
    Startflag = 0;
    Start_i = 1;
    %To trim the size of the matrix, in order to reduce process time
    for i = 1 : size(gcode,1)
        char = '';
        for j = 1:size(gcode{i},2)
            char = gcode{i}(j);
            if strcmp(char, 'Z')
                Zflag = Zflag + 1;
                if Zflag > 1
                    values = zeros(i-Start_i,6);
                    break;
                else
                    Start_i = i;
                end
            end
        end
        if Zflag > 1
            break;
        end
    end
    clear Zflag i
    
    %To read all value into the matrix 'values'
    for i = 1 : size(values,1)
        %6 float numbers(G,X,Y,Z,E,F) is read into the cell 'input'
        input = sscanf(gcode{i+Start_i},'%*c %f %*c %f %*c %f %*c %f %*c %f %*c %f');
        char = '';
        k = 1;
        for j = 1:size(gcode{i+Start_i},2)
            char = gcode{i+Start_i}(j);
            if strcmp(char, 'G')
                values(i:end,1) = input(k);
                k = k+1;
            elseif strcmp(char, 'X')
                values(i:end,2) = input(k);
                k = k+1;
            elseif strcmp(char, 'Y')
                values(i:end,3) = input(k);
                k = k+1;
            elseif strcmp(char, 'Z')
                values(i:end,4) = input(k);
                k = k+1;
            elseif strcmp(char, 'E')
                values(i:end,5) = input(k);
                k = k+1;
            elseif strcmp(char, 'F')
                values(i:end,6) = input(k);
                k = k+1;
            elseif strcmp(char, ';')
                break;
            end
        end
    end
    clear fileID filename pathname gcode i j k input char
    
    x = values(:,2);
    y = values(:,3);
    e = values(:,5)';
    
     
    
%     for i = 2: size(x,1)-1
%         length = sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2);
%         if floor(length/(LaserVelocity*time_step)) > 0
%             step = floor(length/(LaserVelocity*time_step));
%             dx = (x(i+1)-x(i))/step;
%             dy = (y(i+1)-y(i))/step;
%             for j = size(q,2)+1:size(q,2)+step
%                 q(1,j) = q(1,j-1)+dx;
%                 q(2,j) = q(2,j-1)+dy;
%             end
%             x(i+1)=q(1,j);
%             y(i+1)=q(2,j);
%             
%         else
%             x(i+1) = x(i);
%             y(i+1) = y(i);
%         end
%     end

end

%z = ones(size(x))* (0.15);  %for small Laser
z = ones(size(x))*(1.0); %for laserXL

q=zeros(6,size(x,1));
if fitToWorkspace
    if (max(x)-min(x)) > (max(y)-min(y))  %find the scale between pattern and the workspace
        scale = workspaceSize(1)/(max(x)-min(x));
    else
        scale =  workspaceSize(2)/(max(y)-min(y));
    end
    offsetX = 0 - (max(x)+min(x))/2;  %%find the mid point of the X and translate back to center of the workspace
    offsetY = 0 - (max(y)+min(y))/2;
    q(1,:)=((x+offsetX).*scale+ workspaceCenter(1))'; q(2,:)=((y+offsetY).*scale+ workspaceCenter(2))'; q(3,:)=z';
else
    q(1,:)=x'; q(2,:)=y'; q(3,:)=z';
end

clear filename delimiter formatSpec fileID ans pathname dataArray x y z values Startflag Start_i ext;

clear cuboidSize fitToWorkspace frameSize offsetX offsetY scale shrinkWorkspace workspaceSize

%% Laser end-effector setup
if ~exist('laser','var')
    laser = Laser('COM12');
    laser.initialize();
end
laser.laserOff; %to ensure the laser is off

%% Setup TCPIP feedback
tcpipServer = tcpip('0.0.0.0',55000,'NetworkRole','Server');
% fopen(tcpipServer); %Since the laser have no need to send feedback
%% Setup model
fo = FileOperation(which('initstate.ini'));

%q0 = [workspaceCenter(1) workspaceCenter(2) 0.095+0.006 0 0 0]';  %Small Laser
%q0 = [workspaceCenter(1) workspaceCenter(2) 0.135+0.165 0 0 0]';  %4by4by23 LaserXL
q0 = [1.252 1.693 1.190 0 0 0]';
%q0 = [2.0 2.0 0.27069 0 0 0]';
q_transit_point = [1.252 1.693 1.3 0 0 0]';
%q_transit_point = q0 + [0 0 0.1 0 0 0]';  
distance_safe = 0.1;

% unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
%v_operation = 0.005; %for 0.5W laser
v_operation =0.1; %for 5.5W laser
v_quick = 0.12; %= fast transistion for non-lasering part

time_step = 0.05;
cuttingPWM = 255;  %may need to change if stronger laser used
blend_time_default = 0.001; %used to decide the acceleration

%% Preparing the robot from q0 to q1
if ~exist('exp','var')
    %     exp = PoCaBotExperiment(8,'box_endeffector_frame4by4by23_improvedjoint_middle',time_step);
   exp = PoCaBotExperiment(8,'darwin_office_big',time_step, tcpipServer);
   %exp = PoCaBotExperiment(8,'xs_laser_endeffector_regular',time_step, tcpipServer);
    exp.application_preparation(fo, q0);
    q_temp = q0;
    % lift the laser from the ground
    q_temp(3) = q0(3)+distance_safe;
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q_temp, time_step, 0.5, 0.5, 0.1);
    exp.runTrajectoryDirectly(trajectory);
else
    q_temp = exp.q_present;
    q_temp(3) = q0(3)+distance_safe;
    % lift the laser to prevent collision
    trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_temp, time_step, blend_time_default, blend_time_default, 0.1);
    exp.runTrajectoryDirectly(trajectory);
end

%% Move to the position upright above the start point
q_uprightabove = q(:,1);
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_uprightabove, time_step, 0.1, 0.1, 0.1);
exp.runTrajectoryDirectly(trajectory);
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_uprightabove, time_step, 0.1, 0.1, 0.1);
exp.runTrajectoryDirectly(trajectory);

pause(2);
laser.setLaser(1); %check the position of laser with weak laser

%% Run Trajectory
for i=1:size(q, 2)
    
    %For our own sliced Gcode
        if e(i) == 0
            laser.setLaser(1);
            pause(0.1);
            trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q(:,i), time_step, 0, 0, v_quick);
            exp.runTrajectoryDirectly(trajectory);
            pause(0.5);
        else
            laser.setLaser(cuttingPWM);
            trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q(:,i), time_step, 0, 0, v_operation);
            exp.runTrajectoryDirectly(trajectory);
        end
        i
end
clear i

% Z = zeros(size(q,1),1);
% count=1;
% tic;
% while (count <= size(q,2))
%     elapsed = toc - time_step;
%     if(elapsed>0)
% exp.model.update(q(:,count),Z,Z,Z);
% % [~, model_temp, ~, ~, ~] = exp.idsim.IDSolver.resolve(q, Z, Z, Z);
% % [offset] = exp.hardwareInterface.getCableOffsetByTensionByMotorAngleError(model_temp.cableForces);
% % exp.hardwareInterface.lengthCommandSend(model_temp.cableLengths ./(1+exp.factor_offset_per_Newton_Meter*model_temp.cableForces) + offset);
%     exp.hardwareInterface.lengthCommandSend(exp.model.cableLengths);
%     tic;
%     count = count + 1;
%     end
% %     exp.l_feedback_traj(:, i) = exp.hardwareInterface.lengthFeedbackRead;
% end
laser.setLaser(1);

%% Move back to initial point and terminate
trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_transit_point, time_step, 0.1, 0.1, 0.1);
exp.runTrajectoryDirectly(trajectory);
%exp.application_termination();
%clear;
laser.laserOff;
fclose(tcpipServer);
fprintf('Laser Cutting Done:)\n');


% %% Extract useful info from csv
% [filename, pathname] = uigetfile('*.csv;','CSV file (*.csv)','Pick a CSV path');
% fileID = fopen(strcat(pathname,filename),'r');
% delimiter = ',';
% formatSpec = '%f%f%f%f%[^\n\r]';
% dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);
% fclose(fileID);
% x = dataArray{1:1}/1000;
% y = dataArray{2:2}/1000;
% %z = ones(size(x))* (0.138+0.393+0.06);
% z = ones(size(x))* (0.24925+0.0507);
% clearvars filename delimiter formatSpec fileID ans pathname dataArray;
%
% % [filename, pathname] = uigetfile({'*.gco;*.gcode;*.g;','Gcode Files (*.gco,*.gcode,*.g)'},'Pick a file');
% % fileID = fopen(strcat(pathname,filename),'r');
% % gcode = textscan(fileID, '%s','Delimiter', '\r');
% % gcode = gcode{:,1};
% % Zflag = 0;
% % Startflag = 0;
% % Start_i = 1;
% % %To trim the size of the matrix, in order to reduce process time
% % for i = 1 : size(gcode,1);
% %     char = '';
% %     for j = 1:size(gcode{i},2)
% %         char = gcode{i}(j);
% %         if strcmp(char, 'Z')
% %             Zflag = Zflag + 1;
% %         end
% %         if (Startflag < 2 && strcmp(char, 'E'))
% %             Start_i = i;
% %             Startflag = Startflag + 1;
% %         end
% %     end
% %     if Zflag > 1
% %         values = zeros(i-Start_i,6);
% %         break;
% %     end
% % end
% % clear Zflag i
% %
% % %To read all value into the matrix 'values'
% % for i = 1 : size(values,1)
% %     %6 float numbers(G,X,Y,Z,E,F) is read into the cell 'input'
% %     input = sscanf(gcode{i+Start_i},'%*c %f %*c %f %*c %f %*c %f %*c %f %*c %f');
% %     char = '';
% %     k = 1;
% %     for j = 1:size(gcode{i+Start_i},2)
% %         char = gcode{i+Start_i}(j);
% %         if strcmp(char, 'G')
% %             values(i:end,1) = input(k);
% %             k = k+1;
% %         elseif strcmp(char, 'X')
% %             values(i:end,2) = input(k)/1000+2;
% %             k = k+1;
% %         elseif strcmp(char, 'Y')
% %             values(i:end,3) = input(k)/1000+2;
% %             k = k+1;
% %         elseif strcmp(char, 'Z')
% %             values(i:end,4) = input(k)/1000;
% %             k = k+1;
% %         elseif strcmp(char, 'E')
% %             values(i:end,5) = input(k);
% %             k = k+1;
% %         elseif strcmp(char, 'F')
% %             values(i:end,6) = input(k);
% %             k = k+1;
% %         elseif strcmp(char, ';')
% %             break;
% %         end
% %     end
% % end
% % clear fileID filename pathname gcode i j k input char
% %
% % x = values(:,2);
% % y = values(:,3);
% % z = ones(size(x))* (0.138+0.393+0.06);
%
% % q = [x(1);y(1);z(1);0;0;0];
% % LaserVelocity = 0.12;%%0.003;
% % time_step = 0.05;
% % for i = 2: size(x,1)-1
% %     length = sqrt((x(i+1)-x(i))^2+(y(i+1)-y(i))^2);
% %     if floor(length/(LaserVelocity*time_step)) > 0
% %         step = floor(length/(LaserVelocity*time_step));
% %         dx = (x(i+1)-x(i))/step;
% %         dy = (y(i+1)-y(i))/step;
% %     for j = size(q,2)+1:size(q,2)+step
% %         q(1,j) = q(1,j-1)+dx;
% %         q(2,j) = q(2,j-1)+dy;
% %     end
% %     x(i+1)=q(1,j);
% %     y(i+1)=q(2,j);
% %
% %     else
% %         x(i+1) = x(i);
% %         y(i+1) = y(i);
% %     end
% % end
% % q (3,:) = z(1);
% %% Laser end-effector setup
% if ~exist('gripper','var')
%     gripper = Gripper('COM12');
%     gripper.initialize();
% end
%
% gripper.setLaser(1);
%
% % %% Setup TCPIP feedback
%  tcpipServer = tcpip('0.0.0.0',55000,'NetworkRole','Server');
% %
% %
% % fopen(tcpipServer);
% %
% %
%
% %% Setup model
% fo = FileOperation(which('initstate.ini'));
% % 0.138+0.393
% %q0 = [2.0 2.0 0.138+0.393+0.015 0 0 0]';
% %q0 = [2.0 2.0 0.138+0.111 0 0 0]';
% q0 = [2.0 2.0 0.24925 0 0 0]';
% q_transit_point = [2.0 2.0 1.0 0 0 0]';
% distance_safe = 0.1;
% v_max = 0.02;%0.02; % unit: m/s For maximum: 200*0.229/60Rev/s<=>0.763Rev/s*0.1903m/Rev = 0.145m/s
% v_quick = 0.12; %= 0.07;
% time_step = 0.05;
% blend_time_default = 0.001; %used to decide the acceleration
% blend_time_placing = 1.5;
%
% %% Preparing the robot from q0 to q1
% if ~exist('exp','var')
% %     exp = PoCaBotExperiment(8,'box_endeffector_frame4by4by23_improvedjoint_middle',time_step);
%     exp = PoCaBotExperiment(8,'box_endeffector_frame4by4by23_improvedjoint_middle',time_step, tcpipServer);
%
%     exp.application_preparation(fo, q0);
%     q_temp = q0;
%     % lift the gripper from the ground
%     q_temp(3) = q0(3)+distance_safe;
%     trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(q0, q_temp, time_step, 0.5, 0.5, 0.1);
%     exp.runTrajectoryDirectly(trajectory);
% else
%     q_temp = exp.q_present;
%     q_temp(3) = q0(3)+distance_safe;
%     % lift the gripper to prevent collision
%     trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_temp, time_step, blend_time_default, blend_time_default, 0.1);
%     exp.runTrajectoryDirectly(trajectory);
% end
%
% %% Move to the position upright above the start point
% q_uprightabove = [x(1) y(1) z(1) 0 0 0]';
% trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_uprightabove, time_step, 0.1, 0.1, 0.1);
% exp.runTrajectoryDirectly(trajectory);
% gripper.setLaser(2);
%
%
% %% Run Trajectory
% for i=1:size(x, 1)-1
% %     if values(i,5) == 0
% % %       gripper.setLaser(1);
% % %         pause(1);
% % %         gripper.setLaser(1);
% %         q = [x(i) y(i) z(i)+0.05 0 0 0]';
% %         trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q, time_step, 0, 0, v_quick);
% %         exp.runTrajectoryDirectly(trajectory);
% %         gripper.setLaser(2);
% %     else
% %         gripper.setLaser(2);
% %         q = [x(i) y(i) z(i) 0 0 0]';
% %         trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q, time_step, 0, 0, v_quick);
% %         exp.runTrajectoryDirectly(trajectory);
% %         q = [x(i) y(i) z(i) 0 0 0]';
% %         trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q, time_step, 0, 0, v_max);
% %         exp.runTrajectoryDirectly(trajectory);
% %     end
% q = [x(i) y(i) z(i)+0.05 0 0 0]';
%         trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q, time_step, 0, 0, v_max);
%         exp.runTrajectoryDirectly(trajectory);
% % fprintf(tcpipServer,'(%f,%f,%f,%f,%f,%f)', exp.fksolver.model.q);
% end
% clear i
% % Z = zeros(size(q,1),1);
% % count=1;
% % tic;
% % while (count <= size(q,2))
% %     elapsed = toc - time_step;
% %     if(elapsed>0)
% % exp.model.update(q(:,count),Z,Z,Z);
% % % [~, model_temp, ~, ~, ~] = exp.idsim.IDSolver.resolve(q, Z, Z, Z);
% % % [offset] = exp.hardwareInterface.getCableOffsetByTensionByMotorAngleError(model_temp.cableForces);
% % % exp.hardwareInterface.lengthCommandSend(model_temp.cableLengths ./(1+exp.factor_offset_per_Newton_Meter*model_temp.cableForces) + offset);
% %     exp.hardwareInterface.lengthCommandSend(exp.model.cableLengths);
% %     tic;
% %     count = count + 1;
% %     end
% % %     exp.l_feedback_traj(:, i) = exp.hardwareInterface.lengthFeedbackRead;
% % end
% gripper.setLaser(1);
%
% %% Move back to initial point and terminate
% trajectory = PoCaBotExperiment.generateTrajectoryParabolicBlend(exp.q_present, q_transit_point, time_step, 0.1, 0.1, 0.1);
% exp.runTrajectoryDirectly(trajectory);
% %exp.application_termination();
% %clear;
% fclose(tcpipServer);
% fprintf('Laser Cutting Done:)\n');