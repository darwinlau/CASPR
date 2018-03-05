model_config = DevModelConfig('CU-Brick');
% Load the SystemKinematics object from the XML
modelObj = model_config.getModel('demo_causewaybay');
cables = modelObj.cableModel.cables(:);
co_frame4 = cables{4}.attachments{1}.r_OA;
co_ee4_r = cables{4}.attachments{2}.r_GA;
co_frame6 = cables{6}.attachments{1}.r_OA;
co_ee6_r = cables{6}.attachments{2}.r_GA;
co_frame8 = cables{8}.attachments{1}.r_OA;
co_ee8_r = cables{8}.attachments{2}.r_GA;

frame_size = [3.1,2.5,2.8]';
brick_size = [0.12,0.06,0.035]';
d_vertical_offset = 0.3;
height_ee = 0.3025 + d_vertical_offset;
% x,y,z is the coordinate of the bricks
x = 0.2:0.02:frame_size(1)/2;
y = 0.2:0.02:frame_size(2)/2;
interval_z = 0.02;
z = 0:interval_z:frame_size(3);

%% Output the coordinates for all attachment point
co_frame = [];
co_ee_r= [];
for i=1:8
co_frame = [co_frame cables{i}.attachments{1}.r_OA];
co_ee_r = [co_ee_r cables{i}.attachments{2}.r_GA];
fprintf('%d &',i);
fprintf('%.3f &', cables{i}.attachments{1}.r_OA');
fprintf('%.3f &', cables{i}.attachments{2}.r_OA');
fprintf('\\\\');
fprintf('\n');
end

%% Computing the collision free area W_y.
flag_geo_y = zeros(length(x),length(y),length(z));

for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            co_brick = [x(i),y(j),z(k)]';
            co_ee = [co_brick(1),co_brick(2),co_brick(3)+height_ee-brick_size(3)]';
            co_ee6 = co_ee + co_ee6_r;
            co_ee4 = co_ee + co_ee4_r;
            
            slope = co_ee4 - co_frame4;
            % The height of the cable over (frame_size(1)-x,y), with x,y from co_brick
            hxy = slope(3)/(-slope(1))*(co_brick(1)-co_frame6(1))+co_frame6(3);
            if(hxy>co_brick(3))
                flag_geo_y(i,j,k) = 1;
            end
        end
    end
end
[I1,I2,I3] = ind2sub(size(flag_geo_y),find(flag_geo_y));
figure;
plot3(x(I1),y(I2),z(I3));xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
grid on;

%% Computing the collision free area W_x.
flag_geo_x = zeros(length(x),length(y),length(z));

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
                flag_geo_x(i,j,k) = 1;
            end
        end
    end
end
[I1,I2,I3] = ind2sub(size(flag_geo_x),find(flag_geo_x));
figure;
plot3(x(I1),y(I2),z(I3));xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
grid on;

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
plot3(x(I1),y(I2),z(I3),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
grid on;

%% calculate the force and save the data in force_set
id_objective_Quad = IDObjectiveMinQuadCableForce(ones(modelObj.numActuatorsActive,1));
id_solver_Quad = IDSolverQuadProg(modelObj, id_objective_Quad, ID_QP_SolverType.MATLAB);
idsim_Quad = InverseDynamicsSimulator(modelObj, id_solver_Quad);
flag_dynamics = zeros(length(x),length(y),length(z));
force_set = cell(size(flag_dynamics));
distance_safe = 0.1;
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
            co_brick = [x(i),y(j),z(k)]';
            co_ee = [co_brick(1),co_brick(2),co_brick(3)+height_ee+distance_safe]';
            
            q = [co_ee;0;0;0];
            modelObj.update(q, zeros(6,1), zeros(6,1),zeros(6,1));
            [~, model_temp, ~, ~, ~] = idsim_Quad.IDSolver.resolve(q, zeros(6,1), zeros(6,1), zeros(idsim_Quad.model.numDofs,1));
            
            force_set{i,j,k} = model_temp.cableForces;
            if(any(model_temp.cableForces<0))
                flag_dynamics(i,j,k) = 0;
            else
                flag_dynamics(i,j,k) = 1;
            end
        end
    end
end
if(~exist('forceset.mat','file'))
    save('forceset.mat','force_set');
end
figure;
[I1,I2,I3] = ind2sub(size(flag_dynamics),find(flag_dynamics));
plot3(x(I1),y(I2),z(I3));xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
grid on;

%% pick up points with force constraints
force_limit = 50;
flag_dynamics_forcelimit = zeros(length(x),length(y),length(z));
for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            if(max(force_set{i,j,k})<=force_limit && all(force_set{i,j,k}>=0))
                flag_dynamics_forcelimit(i,j,k) = 1;
            end
        end
    end
end

% flag_dynamics_forcelimit offset operation
d_vertical_n = ceil(d_vertical_offset/interval_z);
if(d_vertical_n>=1)
    for i = 1:length(x)
        for j = 1:length(y)
            for k = 1:length(z) - d_vertical_n
                flag_dynamics_forcelimit(i,j,k) = flag_dynamics_forcelimit(i,j,k+d_vertical_n);
            end
        end
    end
    flag_dynamics_forcelimit(:,:,length(z)-d_vertical_n+1:length(z)) = 0;
end

[I1,I2,I3] = ind2sub(size(flag_dynamics_forcelimit),find(flag_dynamics_forcelimit));

% figure;
% plot3(x(I1),y(I2),z(I3),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% % plot(x(I1),z(I3),'.');xlabel('x');ylabel('z');
% title(sprintf('with force limit %dN',force_limit));
% grid on;

% Get the hull of the workspace
fig_integral = figure;

PointSet1 = [x(I1)',y(I2)',z(I3)'];
PointSet2 = [x(I1)',frame_size(2)-y(I2)',z(I3)'];
PointSet3 = [frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)'];
PointSet4 = [frame_size(1)-x(I1)',y(I2)',z(I3)'];
PointSet = [PointSet1;PointSet2;PointSet3;PointSet4];

subpointset_integral = PointSet;
K = convhulln(subpointset_integral);
set(fig_integral, 'Colormap',ones(64,1)*[0.94,0.94,0.94]);
trisurf(K, subpointset_integral(:,1),subpointset_integral(:,2),subpointset_integral(:,3));
lightsource = light;
set(lightsource, 'Position',[0 0 100]);
set(fig_integral.CurrentAxes.Children(2), 'LineStyle','none');

xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
axis equal;
axis([0 frame_size(1) 0 frame_size(2) 0 frame_size(3)]);
%% void collision between the cable and the calibration setup
% center_calb = [0.6,1.25]';
% width_calb = 0.30;
% flag_calb = flag_dynamics_forcelimit;
% for i = 1:length(x)
%     for j = 1:length(y)
%         for k = 1:length(z)
%             if(flag_calb(i,j,k) == 1)
%                 if(x(i)<=center_calb(1)+width_calb/2+0.23/2)
%                     flag_calb(i,j,k) = 0;
%                 else
%                     co_brick = [x(i),y(j),z(k)]';
%                     co_ee = [co_brick(1),co_brick(2),co_brick(3)+height_ee-brick_size(3)]';
%                     co_ee8 = co_ee + co_ee8_r;
%                     
%                     slope = co_ee8 - co_frame8;
%                     % The offset in x direction given y=frame_size(2)/2+width_calb/2
%                     ox = -slope(1)/slope(2)*(frame_size(2)/2-co_frame6(2)-width_calb/2)+co_frame6(1);
%                     if(ox<=center_calb(1)+width_calb/2)
%                         flag_calb(i,j,k) = 0;
%                     end
%                 end
%             end
%         end
%     end
% end

%% output the data into *.txt file
% [I1,I2,I3] = ind2sub(size(flag_dynamics_forcelimit),find(flag_dynamics_forcelimit));
% filename_txt = 'C:\Users\trist\OneDrive\Desktop\workspace.txt';
% fileID = fopen(filename_txt,'w');
% fprintf(fileID,'%.3f %.3f %.3f\n', [frame_size(1)-x(I1)',y(I2)',z(I3)']');
% fprintf(fileID,'%.3f %.3f %.3f\n', [frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)']');
% fprintf(fileID,'%.3f %.3f %.3f\n', [x(I4)',y(I5)',z(I6)']');
% fprintf(fileID,'%.3f %.3f %.3f\n', [x(I4)',frame_size(2)-y(I5)',z(I6)']');
% fclose(fileID);

% filename_csv = 'C:\Users\trist\OneDrive\Desktop\workspace.csv';
% dlmwrite(filename_csv,[frame_size(1)-x(I1)',y(I2)',z(I3)'],'precision','%.3f');
% dlmwrite(filename_csv,[frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)'],'-append','precision','%.3f');
% dlmwrite(filename_csv,[x(I4)',y(I5)',z(I6)'],'-append','precision','%.3f');
% dlmwrite(filename_csv,[x(I4)',frame_size(2)-y(I5)',z(I6)'],'-append','precision','%.3f');
% figure;
% plot3(frame_size(1)-x(I1),y(I2),z(I3),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% hold on;
% plot3(frame_size(1)-x(I1),frame_size(2)-y(I2),z(I3),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% plot3(x(I4),y(I5),z(I6),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% plot3(x(I4),frame_size(2)-y(I5),z(I6),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% title('workspace');
% grid on;axis equal;
% hold off;


%% test the data
% data = load(filename_csv);
% figure;
% plot3(data(:,1),data(:,2),data(:,3),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% title('workspace');
% axis equal;
% grid on;
if(~exist('workspace_v2.mat','file'))
    save('workspace_v2.mat');
end


%% Consider all the criteria 
% flag_geo_x
% flag_geo_y
% flag_geo_v2
% flag_dynamics_forcelimit
flag_integral = zeros(length(x),length(y),length(z));
for i = 1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            if(flag_geo_x(i,j,k) && flag_geo_v2(i,j,k) && flag_dynamics_forcelimit(i,j,k))
                flag_integral(i,j,k) = 1;
            end
        end
    end
end

% figure;
% [I1,I2,I3] = ind2sub(size(flag_integral),find(flag_integral));
% plot3(x(I1),y(I2),z(I3),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% % plot(x(I1),z(I3),'.');xlabel('x');ylabel('z');
% title(sprintf('with force limit %dN',force_limit));
% grid on;

% catenate the four parts
% flag_integral_i are named with a clock-wise order
flag_integral_1 = flag_integral;
flag_integral_2 = flip(flag_integral,1);
flag_integral_3 = flip(flag_integral_2,2);
flag_integral_4 = flip(flag_integral,2);

flag_integral_all_1 = [flag_integral_1; flag_integral_2];
flag_integral_all_2 = [flag_integral_4; flag_integral_3];
flag_integral_all = [flag_integral_all_1 flag_integral_all_2];
X = [x, frame_size(1) - flip(x)];
Y = [y, frame_size(2) - flip(y)];
Z = z;

% figure;
% [I1,I2,I3] = ind2sub(size(flag_integral_4),find(flag_integral_4));
% plot3(X(I1),Y(I2),Z(I3),'.');xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% % plot(x(I1),z(I3),'.');xlabel('x');ylabel('z');
% grid on;

% Get the hull of the point cloud
flag_integral_all_temp = ones(size(flag_integral_all));

for i = 2:size(flag_integral_all,1)-1
    for j = 2:size(flag_integral_all,2)-1
        for k = 2:size(flag_integral_all,3)-1
            if(flag_integral_all(i-1,j,k) &&...
                    flag_integral_all(i+1,j,k) &&...
                    flag_integral_all(i,j-1,k) &&...
                    flag_integral_all(i,j+1,k) &&...
                    flag_integral_all(i,j,k-1) &&...
                    flag_integral_all(i,j,k+1))
                flag_integral_all_temp(i,j,k) = 0;
            end
        end
    end
end

flag_integral_all_hull = flag_integral_all_temp.*flag_integral_all;
[I1,I2,I3] = ind2sub(size(flag_integral_all_hull),find(flag_integral_all_hull));
%figure;
C = ones(1,3)*0.25;
plot3(X(I1),Y(I2),Z(I3),'.','MarkerSize',5,'MarkerEdgeColor',C);
xlabel('X(m)'); 
ylabel('Y(m)'); 
zlabel('Z(m)');
% plot(x(I1),z(I3),'.');xlabel('x');ylabel('z');
grid on;
% axis equal;
axis([0 frame_size(1) 0 frame_size(2) 0 frame_size(3)-0.8]);
hold on;
%figure;
%C = ones(1,3)*0.55;
%pcshow([X(I1)',Y(I2)',Z(I3)'],C);

%% Plot  Visualize Draw
% flag_integral_all_hull
direction = 2;%1-x 2-y 3-z
sample_law = 4;
C = ones(1,3)*0.25;
flag_integral_all_hull = flag_integral_all_hull_600;
flag_integral_all_hull_sample = zeros(size(flag_integral_all_hull));

if(direction == 1)
    flag_integral_all_hull_sample(1:sample_law:end,:,:) = flag_integral_all_hull(1:sample_law:end,:,:);
elseif(direction == 2)
    flag_integral_all_hull_sample(:,1:sample_law:end,:) = flag_integral_all_hull(:,1:sample_law:end,:);
elseif(direction == 3)
    flag_integral_all_hull_sample(:,:,1:sample_law:end) = flag_integral_all_hull(:,:,1:sample_law:end);
else
    error('wrong direction');
end

[I1,I2,I3] = ind2sub(size(flag_integral_all_hull_sample),find(flag_integral_all_hull_sample));
%figure;
plot3(X(I1)+0.01,Y(I2),Z(I3),'.','MarkerSize',5,'MarkerEdgeColor',C);
xlabel('X(m)'); 
ylabel('Y(m)'); 
zlabel('Z(m)');
% plot(x(I1),z(I3),'.');xlabel('x');ylabel('z');
grid on;
hold on;
% pcshow([X(I1)',Y(I2)',Z(I3)'],C);















































%% A more general way to visulize the data.
% flag_visualization = flag_dynamics_forcelimit;
% for i = 1:length(x)
%     for j = 1:length(y)
%         m = find(flag_visualization(i,j,:)>0,true,'last');
%         flag_visualization(i,j,1:m-1) = false;
%     end
% end
% [I1,I2,I3] = ind2sub(size(flag_visualization),find(flag_visualization));
% 
% PointSet1_Temp = [x(I1)',y(I2)',z(I3)'];
% PointSet2_Temp = [x(I1)',frame_size(2)-y(I2)',z(I3)'];
% PointSet3_Temp = [frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)'];
% PointSet4_Temp = [frame_size(1)-x(I1)',y(I2)',z(I3)'];
% PointSet_integral = [PointSet1_Temp;PointSet2_Temp;PointSet3_Temp;PointSet4_Temp];
% fig_integral = figure;
% T = delaunay(PointSet_integral(:,1),PointSet_integral(:,2),PointSet_integral(:,3));
% surfObj = trisurf(T,PointSet_integral(:,1),PointSet_integral(:,2),PointSet_integral(:,3),'FaceColor','interp','FaceLighting','phong');
% set(surfObj, 'LineStyle','none');
% lightsource = light;
% set(lightsource, 'Position',[0 0 -100]);
% set(fig_integral, 'Colormap',ones(64,1)*[0.94,0.94,0.94]);
% xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% axis equal;
% axis([0 frame_size(1) 0 frame_size(2) 0 frame_size(3)]);
% grid on;



%% The way to visulize the surface with convhulln to get the hull
% flag_visualization = flag_geo_x;
% [I1,I2,I3] = ind2sub(size(flag_visualization),find(flag_visualization));
% PointSet1_Temp = [x(I1)',y(I2)',z(I3)'];
% PointSet2_Temp = [x(I1)',frame_size(2)-y(I2)',z(I3)'];
% PointSet3_Temp = [frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)'];
% PointSet4_Temp = [frame_size(1)-x(I1)',y(I2)',z(I3)'];
% PointSet_integral = {[PointSet1_Temp; PointSet2_Temp;PointSet3_Temp; PointSet4_Temp]};
% % PointSet_integral = [PointSet_integral,{[PointSet3_Temp; PointSet4_Temp]}];
% 
% fig_integral = figure;
% hold on;
% for i = 1:length(PointSet_integral)
%     subpointset_integral = PointSet_integral{i};
%     K = convhulln(subpointset_integral);
%     surf_instance = trisurf(K, subpointset_integral(:,1),subpointset_integral(:,2),subpointset_integral(:,3));
%     set(surf_instance, 'LineStyle','none');
% end
% lightsource = light;
% set(lightsource, 'Position',[0 0 100]);
% set(fig_integral, 'Colormap',ones(64,1)*[0.94,0.94,0.94]);
% xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% axis equal;
% axis([0 frame_size(1) 0 frame_size(2) 0 frame_size(3)]);
% grid on;
%hold off;


%% The way to visulize the surface with point cloud
flag_visualization = flag_integral;
[I1,I2,I3] = ind2sub(size(flag_visualization),find(flag_visualization));
PointSet1_Temp = [x(I1)',y(I2)',z(I3)'];
PointSet2_Temp = [x(I1)',frame_size(2)-y(I2)',z(I3)'];
PointSet3_Temp = [frame_size(1)-x(I1)',frame_size(2)-y(I2)',z(I3)'];
PointSet4_Temp = [frame_size(1)-x(I1)',y(I2)',z(I3)'];
PointSet_integral = {[PointSet1_Temp; PointSet2_Temp;PointSet3_Temp; PointSet4_Temp]};

% PointSet_integral = [PointSet_integral,{[PointSet3_Temp; PointSet4_Temp]}];

fig_integral = figure;
c = ones(64,3);
for i = 1:64
    c(i,:) = i/80 + 0.1;
end
colormap(c);
hold on;
for i = 1:length(PointSet_integral)
    subpointset_integral = PointSet_integral{i};
    subpointset_integral(:,1) = subpointset_integral(:,1) + 0.01;
    pcshow(subpointset_integral,'MarkerSize',10);
end
% set(fig_integral, 'Colormap',ones(64,1)*[0.5,0.5,0.5]);
xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% axis equal;
axis([0 frame_size(1) 0 frame_size(2) 0 frame_size(3)-0.8]);
view(-35.9502,47.9521);
grid on;
%hold off;

%set(fig_integral,'Units','Inches');
%pos = get(fig_integral,'Position');
%pos = [0 0 pos(3), pos(4)];
%set(fig_integral,'Position',pos);
%set(fig_integral,'PaperPosition',pos);
%set(fig_integral,'PaperPositionMode','Manual','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
%print(fig_integral,'filename','-dpdf','-r0')

%% further adjust the figure
% fig_integral = gcf;
% fig_axes = gca;
% for i = 1:length(fig_axes.Children)
%     cdata = fig_axes.Children(i).CData;
%     fig_axes.Children(i).CData = ones(size(cdata))*i;
%     set(fig_axes.Children(i),'MarkerEdgeAlpha',0.1);
% end
% legend(fig_axes.Children,'d = 0.3m','d=0.5m');