% A container class to hold workspace result for trajectory analysis and
% plotting function of it
%
% Author        : Paul Cheng
% Created       : 2019
% Description    : This class contains the known information obtained
% through analysis. Plot function supported

classdef TrajectoryWorkspace < handle
    properties (SetAccess = protected)
        model
    end
    
    properties
        trajectories                        % Cell array
    end
    
    methods
        function tw = TrajectoryWorkspace(model)
            tw.model = model;
        end
        % A function to draw the moving animation
        function graph = PlotAnimation(obj,trajectory_number)
            if max(size(obj.trajectories)) < trajectory_number
                CASPR_log.Error('Input number excess the number of trajectories');
            end
            if isempty(obj.trajectories{trajectory_number}.feasible_time_range)
                CASPR_log.Error('No feasible time range');
            end
            PlotPath(obj,trajectory_number);
            
            if iscell(obj.trajectories{trajectory_number}.feasible_time_range)
                theta = obj.trajectories{trajectory_number}.conditions{1}.theta;
%                 traj_feasible_time_range = round(obj.trajectories{trajectory_number}.feasible_time_range{1}*tan(theta/2),9);
                traj_feasible_time_range = round(obj.trajectories{trajectory_number}.feasible_time_range{1},9);
                trajectory_type = 1;                
                default_time_range = obj.trajectories{trajectory_number}.default_time_range;
%                 default_time_range = obj.trajectories{trajectory_number}.default_time_range*tan(theta/2);
            else
                theta = [];
                traj_feasible_time_range = round(obj.trajectories{trajectory_number}.feasible_time_range,9);
                trajectory_type = 0;
                default_time_range = obj.trajectories{trajectory_number}.default_time_range;
            end
            
%             time_range = unique([default_time_range,traj_feasible_time_range(:)']);
            time_range = default_time_range;
            
%             time_range = obj.trajectories{trajectory_number}.default_time_range ;
            
            time_step = linspace(time_range(1),time_range(2),100);
            for j = 1:size(time_step,2)
                    traj(:,j) = GetInstantPose(obj.trajectories{trajectory_number},time_step(j),obj.model,trajectory_type,theta)';
            end
                                    
            if ~isempty(time_range)
                obj.trajectories{trajectory_number}.plotting.cable_graph = [];
%                 xlim(1.1*[obj.model.bodyModel.q_min(1),obj.model.bodyModel.q_max(1)]);
%                 ylim(1.1*[obj.model.bodyModel.q_min(2),obj.model.bodyModel.q_max(2)]);
                zlim(1.1*[obj.model.bodyModel.q_min(3),obj.model.bodyModel.q_max(3)]);
                xlim(1.1*[0 3]);
                ylim(1.1*[0 2.5]);
                hold on;
                grid on;
                for i = 1:length(time_step)
                    if ~isempty(obj.trajectories{trajectory_number}.plotting.cable_graph)
                        delete(obj.trajectories{trajectory_number}.plotting.cable_graph);
                    end
                    
                    obj.model.update(traj(:,i), zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1));
                    
                    for j = 1:obj.model.numCables
                        X(j) = obj.model.cableModel.cables{1,j}.attachments{1, 2}.r_OA(1);
                        Y(j) = obj.model.cableModel.cables{1,j}.attachments{1, 2}.r_OA(2);
                        Z(j) = obj.model.cableModel.cables{1,j}.attachments{1, 2}.r_OA(3);
                        
                        cable_exit_point_x(j) = obj.model.cableModel.cables{1,j}.attachments{1, 1}.r_GA(1);
                        cable_exit_point_y(j) = obj.model.cableModel.cables{1,j}.attachments{1, 1}.r_GA(2);
                        cable_exit_point_z(j) = obj.model.cableModel.cables{1,j}.attachments{1, 1}.r_GA(3);
                        
                        drawline_x = [X(j);cable_exit_point_x(j)];
                        drawline_y = [Y(j);cable_exit_point_y(j)];
                        drawline_z = [Z(j);cable_exit_point_z(j)];
                        
                        obj.trajectories{trajectory_number}.plotting.cable_graph(j) = line(drawline_x,drawline_y,drawline_z,'Color','black');
                    end
                    if ~isempty(traj_feasible_time_range)
                        for k = 1:size(traj_feasible_time_range,1)
                            if time_step(i)<= traj_feasible_time_range(k,end) &&...
                                    time_step(i)>= traj_feasible_time_range(k,1)                                
                                detect_range(k) = 1;
                            else
                                
                                detect_range(k) = 0;
                            end
                        end
                    else
                        detect_range = 0;
                    end
                    if any(detect_range == 1)
                        EE_color = 'g';
                        pause_time = 0;
                    else
                        EE_color = [0.7 0.7 0.7];
                        pause_time = 0.005;
                    end
                    
                    if all(Z == 0)
                        obj.trajectories{trajectory_number}.plotting.end_effector = fill(X,Y,EE_color);
                    else
                        obj.trajectories{trajectory_number}.plotting.end_effector  = DrawEndEffector(obj.model,traj(:,i),EE_color);
%                         DT = delaunayTriangulation(X',Y',Z');
%                         cubepoints = DT.Points;
%                         k_c = convexHull(DT);
%                         obj.trajectories{trajectory_number}.plotting.end_effector = trisurf(k_c,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3),...
%                             'FaceColor',EE_color,'EdgeColor','black','FaceAlpha',0.5);%draw the box
                    end
                    drawnow
                    pause(pause_time)
                    undelete_set = round(linspace(0,length(time_step),0),0);
                    undelete_set(1) = 1 ;
                    if i ~= length(time_step)
                        if ~ismember(i,undelete_set)
                        delete(obj.trajectories{trajectory_number}.plotting.end_effector);
                        end
                    end
                end
            end
            
        end
        % A functio to draw path(for paper)
         function [] = PlotTrajectory(obj,trajectory_num)
            if ~isempty(obj.trajectories{trajectory_num}.conditions{1,1}.theta)
                theta = obj.trajectories{trajectory_num}.conditions{1}.theta;
%                 traj_feasible_time_range = round(obj.trajectories{trajectory_num}.feasible_time_range{1}*tan(theta/2),9);
                traj_feasible_time_range = round(obj.trajectories{trajectory_num}.feasible_time_range{1},9);
                trajectory_type = 1;                
                default_time_range = obj.trajectories{trajectory_num}.default_time_range;
                
%                 default_time_range = obj.trajectories{trajectory_num}.default_time_range*tan(theta/2);
            else
                theta = [];
                traj_feasible_time_range = round(obj.trajectories{trajectory_num}.feasible_time_range,9);
                trajectory_type = 0;
                default_time_range = obj.trajectories{trajectory_num}.default_time_range;
            end
            
            time_range = unique([default_time_range,traj_feasible_time_range(:)']);
            
            for i = 1:size(time_range,2)-1
                
                time_step = linspace(time_range(i),time_range(i+1),100);
                for j = 1:size(time_step,2)
                    q(j,:) = GetInstantPose(obj.trajectories{trajectory_num},time_step(j),obj.model,trajectory_type,theta);
                end
                hold on;
                view(3)
                
                if any(ismember(traj_feasible_time_range(:,1),time_range(i))) &&...
                        any(ismember(traj_feasible_time_range(:,2),time_range(i+1)))
                    obj.trajectories{trajectory_num}.plotting.path(i,:) = plot3(q(:,1),q(:,2),q(:,3),'Color','k','LineWidth',1);
                else
                    obj.trajectories{trajectory_num}.plotting.path(i,:) = plot3(q(:,1),q(:,2),q(:,3),'Color','m','LineWidth',1);
%                      obj.trajectories{trajectory_num}.plotting.path(i,:) = plot3(q(:,1),q(:,2),q(:,3),'--','Color',[0.7 0.7 0.7],'LineWidth',5);
                end
            end
        end
    end
    
    methods(Access = private)
        %% A function to draw path
        function [] = PlotPath(obj,trajectory_num)
            if ~isempty(obj.trajectories{trajectory_num}.conditions{1,1}.theta)
                theta = obj.trajectories{trajectory_num}.conditions{1}.theta;
%                 traj_feasible_time_range = round(obj.trajectories{trajectory_num}.feasible_time_range{1}*tan(theta/2),9);
                traj_feasible_time_range = round(obj.trajectories{trajectory_num}.feasible_time_range{1},9);
                trajectory_type = 1;                
                default_time_range = obj.trajectories{trajectory_num}.default_time_range;
                
%                 default_time_range = obj.trajectories{trajectory_num}.default_time_range*tan(theta/2);
            else
                theta = [];
                traj_feasible_time_range = round(obj.trajectories{trajectory_num}.feasible_time_range,9);
                trajectory_type = 0;
                default_time_range = obj.trajectories{trajectory_num}.default_time_range;
            end
            
            time_range = unique([default_time_range,traj_feasible_time_range(:)']);
            
            for i = 1:size(time_range,2)-1
                
                time_step = linspace(time_range(i),time_range(i+1),100);
                for j = 1:size(time_step,2)
                    q(j,:) = GetInstantPose(obj.trajectories{trajectory_num},time_step(j),obj.model,trajectory_type,theta);
                end
                hold on;
                view(3)
                
                if any(ismember(traj_feasible_time_range(:,1),time_range(i))) &&...
                        any(ismember(traj_feasible_time_range(:,2),time_range(i+1)))
                    obj.trajectories{trajectory_num}.plotting.path(i,:) = plot3(q(:,1),q(:,2),q(:,3),'Color','k','LineWidth',5);
                else
                    obj.trajectories{trajectory_num}.plotting.path(i,:) = plot3(q(:,1),q(:,2),q(:,3),'Color','r','LineWidth',5);
%                      obj.trajectories{trajectory_num}.plotting.path(i,:) = plot3(q(:,1),q(:,2),q(:,3),'--','Color',[0.7 0.7 0.7],'LineWidth',5);
                end
            end
        end
         
    end
    
end
% A function to get single pose information
function [pose] = GetInstantPose(trajectory,time,model,trajectory_type,theta)
if trajectory_type == 0
    for i = 1:size(trajectory.parametric_functions,2)
        pose(i) = feval(trajectory.parametric_functions{i},time);
    end
else
    for j = 1:size(trajectory.parametric_functions,2)
        if model.bodyModel.q_dofType(j) == 'TRANSLATION'
            pose(j,:) = feval(trajectory.parametric_functions{j},time);
        else
            orientation_index = find(ismember(model.bodyModel.q_dofType,'ROTATION'));
            for ii = 1:size(orientation_index,2)
                rad_Q(:,ii) = feval(trajectory.parametric_functions{orientation_index(ii)},trajectory.default_time_range);
            end
            q_s = angle2quat(rad_Q(1,1),rad_Q(1,2),rad_Q(1,3),'XYZ');
            q_e = angle2quat(rad_Q(2,1),rad_Q(2,2),rad_Q(2,3),'XYZ');
            k = tan(theta/2)/trajectory.default_time_range(2);
            if theta~=0
                tau = atan(k*time)*2/theta;
            else
                tau = 0;
            end
            q_t = quatinterp(q_s,q_e,tau,'slerp');
            [r1, r2, r3] = quat2angle(q_t, 'XYZ');
            pose(orientation_index,:) = [r1, r2, r3]';
            break;
        end
    end    
end
end

function [drawbox] = DrawEndEffector(model,traj,EE_color)

model.update(zeros(model.numDofs,1), zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
for j = 1:model.numCables
X(j) = model.cableModel.cables{1,j}.attachments{1, 2}.r_OA(1);
Y(j) = model.cableModel.cables{1,j}.attachments{1, 2}.r_OA(2);
Z(j) = model.cableModel.cables{1,j}.attachments{1, 2}.r_OA(3);
end
length_x = (max(X)-min(X));
length_y = (max(Y)-min(Y));
length_z = (max(Z)-min(Z));

ver_x = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
ver_y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
ver_z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

X_face = length_x*(ver_x-0.5) + traj(1);
Y_face = length_y*(ver_y-0.5) + traj(2);
Z_face = length_z*(ver_z-0.5) + traj(3);

drawbox = fill3(X_face,Y_face,Z_face,EE_color,'FaceAlpha',0);
rotate(drawbox,[1,0,0],[rad2deg(traj(4))],[traj(1:3)]);
rotate(drawbox,[0,1,0],[rad2deg(traj(5))],[traj(1:3)]);
rotate(drawbox,[0,0,1],[rad2deg(traj(6))],[traj(1:3)]);

end
