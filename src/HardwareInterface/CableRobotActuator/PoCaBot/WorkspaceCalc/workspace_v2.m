model_config = DevModelConfig('CU-Brick');
% Load the SystemKinematics object from the XML
modelObj = model_config.getModel('demo_causewaybay');
cables = modelObj.cableModel.cables(:);
co_frame6 = cables{6}.attachments{1}.r_OA;
co_ee6_r = cables{6}.attachments{2}.r_GA;
co_frame8 = cables{8}.attachments{1}.r_OA;
co_ee8_r = cables{8}.attachments{2}.r_GA;

frame_size = [3.1,2.5,2.8]';
brick_size = [0.12,0.06,0.035]';
height_ee = 0.3025;
% x,y,z is the coordinate of the bricks
x = 0.2:0.02:frame_size(1)/2;
y = 0.2:0.02:frame_size(2)/2;
z = 0:0.01:frame_size(3)-1.1;
flag_geo = zeros(length(x),length(y),length(z));

for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            co_brick = [x(i),y(j),z(k)]';
            co_ee = [co_brick(1),co_brick(2),co_brick(3)+height_ee-brick_size(3)]';
            co_ee6 = co_ee + co_ee6_r;
            co_ee8 = co_ee + co_ee8_r;
            
            slope = co_ee8 - co_frame8;
            % The height of the cable over (x,frame_size(2)-y), with x,y from co_brick
            hxy = slope(3)/(-slope(2))*(co_brick(2)-co_frame6(2))+co_frame6(3);
            if(hxy>co_brick(3))
                flag_geo(i,j,k) = 1;
            end
        end
    end
end
[I1,I2,I3] = ind2sub(size(flag_geo),find(flag_geo));
figure;
plot3(x(I1),y(I2),z(I3));xlabel('x');ylabel('y');zlabel('z');
grid on;

%% calculate the force and save the data in force_set
id_objective_Quad = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
id_solver_Quad = IDSolverQuadProg(modelObj, id_objective_Quad, ID_QP_SolverType.MATLAB);
idsim_Quad = InverseDynamicsSimulator(modelObj, id_solver_Quad);
flag_dynamics = flag_geo;
force_set = cell(size(flag_dynamics));
distance_safe = 0.05;
% j_start = 1;
% for i = 1:length(x)
%     j_start = 1;
%     for k = 1:length(z)
%         for j = j_start:length(y)
%             if(flag_dynamics(i,j,k) == 1)
%                 co_brick = [x(i),y(j),z(k)]';
%                 co_ee = [co_brick(1),co_brick(2),co_brick(3)+height_ee]';
%                 
%                 q = [co_ee;0;0;0];
%                 modelObj.update(q, zeros(6,1), zeros(6,1),zeros(6,1));
%                 [~, model_temp, ~, ~, ~] = idsim_Quad.IDSolver.resolve(q, zeros(6,1), zeros(6,1), zeros(idsim_Quad.model.numDofs,1));
%                 if(any(model_temp.cableForces<0))
%                     flag_dynamics(i,j,k) = 0;
%                 else
%                     j_start = j;
%                     break;
%                 end
%             end
%         end
%     end
% end

for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            if(flag_dynamics(i,j,k) == 1)
                co_brick = [x(i),y(j),z(k)]';
                co_ee = [co_brick(1),co_brick(2),co_brick(3)+height_ee+distance_safe]';
                
                q = [co_ee;0;0;0];
                modelObj.update(q, zeros(6,1), zeros(6,1),zeros(6,1));
                [~, model_temp, ~, ~, ~] = idsim_Quad.IDSolver.resolve(q, zeros(6,1), zeros(6,1), zeros(idsim_Quad.model.numDofs,1));
                
                force_set{i,j,k} = model_temp.cableForces;
                if(any(model_temp.cableForces<0))
                    flag_dynamics(i,j,k) = 0;
                end
            end
        end
    end
end
save('forceset.mat','force_set')
figure;
[I1,I2,I3] = ind2sub(size(flag_dynamics),find(flag_dynamics));
plot3(x(I1),y(I2),z(I3));xlabel('x');ylabel('y');zlabel('z');
grid on;

%% pick up points with force constraints
force_limit = 30;
flag_dynamics_forcelimit = flag_dynamics;
for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            if(flag_dynamics_forcelimit(i,j,k) == 1)
                if(max(force_set{i,j,k})>force_limit)
                    flag_dynamics_forcelimit(i,j,k) = 0;
                end
            end
        end
    end
end
figure;
[I1,I2,I3] = ind2sub(size(flag_dynamics_forcelimit),find(flag_dynamics_forcelimit));
plot3(x(I1),y(I2),z(I3),'.');xlabel('x');ylabel('y');zlabel('z');
title(sprintf('with force limit %dN',force_limit));
grid on;

%% void collision between the cable and the calibration setup
center_calb = [0.6,1.25]';
width_calb = 0.30;
flag_calb = flag_dynamics_forcelimit;
for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            if(flag_calb(i,j,k) == 1)
                if(x(i)<=center_calb(1)+width_calb/2+0.23/2)
                    flag_calb(i,j,k) = 0;
                else
                    co_brick = [x(i),y(j),z(k)]';
                    co_ee = [co_brick(1),co_brick(2),co_brick(3)+height_ee-brick_size(3)]';
                    co_ee8 = co_ee + co_ee8_r;
                    
                    slope = co_ee8 - co_frame8;
                    % The offset in x direction given y=frame_size(2)/2+width_calb/2
                    ox = -slope(1)/slope(2)*(frame_size(2)/2-co_frame6(2)-width_calb/2)+co_frame6(1);
                    if(ox<=center_calb(1)+width_calb/2)
                        flag_calb(i,j,k) = 0;
                    end
                end
            end
        end
    end
end

%% output the data into *.txt file
[I1,I2,I3] = ind2sub(size(flag_dynamics_forcelimit),find(flag_dynamics_forcelimit));
[I4,I5,I6] = ind2sub(size(flag_calb),find(flag_calb));
% filename_txt = 'C:\Users\trist\OneDrive\Desktop\workspace.txt';
filename_csv = 'C:\Users\trist\OneDrive\Desktop\workspace.csv';
% fileID = fopen(filename_txt,'w');
% fprintf(fileID,'%.3f %.3f %.3f\n', [frame_size(1)-x(I1)',y(I2)',z(I3)']');
% fprintf(fileID,'%.3f %.3f %.3f\n', [frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)']');
% fprintf(fileID,'%.3f %.3f %.3f\n', [x(I4)',y(I5)',z(I6)']');
% fprintf(fileID,'%.3f %.3f %.3f\n', [x(I4)',frame_size(2)-y(I5)',z(I6)']');
% fclose(fileID);
dlmwrite(filename_csv,[frame_size(1)-x(I1)',y(I2)',z(I3)'],'precision','%.3f');
dlmwrite(filename_csv,[frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)'],'-append','precision','%.3f');
dlmwrite(filename_csv,[x(I4)',y(I5)',z(I6)'],'-append','precision','%.3f');
dlmwrite(filename_csv,[x(I4)',frame_size(2)-y(I5)',z(I6)'],'-append','precision','%.3f');
figure;
plot3(frame_size(1)-x(I1),y(I2),z(I3),'.');xlabel('x');ylabel('y');zlabel('z');
hold on;
plot3(frame_size(1)-x(I1),frame_size(2)-y(I2),z(I3),'.');xlabel('x');ylabel('y');zlabel('z');
plot3(x(I4),y(I5),z(I6),'.');xlabel('x');ylabel('y');zlabel('z');
plot3(x(I4),frame_size(2)-y(I5),z(I6),'.');xlabel('x');ylabel('y');zlabel('z');
title('workspace');
grid on;axis equal;
hold off;


%% test the data
data = load(filename_csv);
figure;
plot3(data(:,1),data(:,2),data(:,3),'.');xlabel('x');ylabel('y');zlabel('z');
title('workspace');
axis equal;
grid on;
save('workspace_v2.mat');

%% consider giving away the occupied central space for the end effector to do calibration and avoid cable-construction collision.
ee_side_length = 0.3;
flag_geo_v2 = zeros(length(x),length(y),length(z));
slope_yx = (frame_size(2)/2-co_frame6(2))/(frame_size(1)/2-co_frame6(1));
for i = 1:length(x)
    for j = 1:length(y)
        if y(j)-co_frame6(2) >= slope_yx*(x(i)-co_frame6(1))
            if(x(i)<frame_size(1)/2-ee_side_length/2)
                flag_geo_v2(i,j,:)=1;
            end
        end
    end
end
[I1,I2,I3] = ind2sub(size(flag_geo_v2),find(flag_geo_v2));
figure;
plot3(x(I1),y(I2),z(I3),'.');xlabel('x');ylabel('y');zlabel('z');
grid on;