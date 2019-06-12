% A container class to hold workspace analysis information for any paths
% Return a time range for valid path
%
% Author        : Paul,Hung Hon Cheng
% Created       : 2018
% Description    : NaN

classdef PathVerification < handle
    properties
        feasible_range = [];                      %the entry and exit pose on the trajectory
        feasible_period = [];                     %valid time range path
        path_type = [];                           %path type: Polynoimal/Poses/Nonpolynomial function
        parametric_functions = [];                %input parametric trajectories
        compute_time = [];
        drawing = [];
%         path_poses = [];                    %all path poses
    end
    properties (Access = private)
        path_time_range = [];
        path_poses = [];                      %all path poses
        estimated_path_polynomials = {};      %estimated parametric path if the polynomial function is not given
        feasible_interval = [];                     %time that entry or exit the WCW
        path_fineness = 80;                   %higher the fineness, slower but more accuray
        time_step = [];                             %time_step
        tolerance = 9;
    end
    
    methods
        % The constructor for the path analysis class
        function FT = PathVerification(model,parametric_functions,path_data,path_type)
            FT@SimulatorBase(model);
            FT.path_type = path_type;
            FT.path_time_range = [0,1];
            FT.parametric_functions = parametric_functions;
            if ~isempty(path_data)
                path_poses =  path_data(1:6,:);
                FT.path_poses = num2cell(path_poses(:,1:end)',2);
                FT.path_time_range = [path_data(end,1),path_data(end,end)];
            end
        end
        %% A function to start the varification
        function run(obj)
            
            tic;
%             if ~isequal(obj.path_type ,'Poses')
            TrajectoryTimeRange(obj,obj.path_type);
%             end
            PosesData(obj);
            J_d = JacobianData(obj);
            count_1 = toc;
            TrajectoryFeasibilityVarification(obj,J_d);
            count_2 = toc;
            obj.compute_time = count_2;
            X = ['Total running time: ',num2str(count_2),' Verification time: ',num2str(count_2-count_1)];
            disp(X)
            figure(1)
            view(3)
            PathAnimation(obj);
        end
        %% A function to get sample points
        function [] = PosesData(obj)
            
            if ~isequal(obj.path_type ,'Poses')
                
                obj.time_step = linspace(obj.path_time_range(1),obj.path_time_range(2),obj.path_fineness);
                
                for i = 1:size(obj.time_step,2)
                    pose{i,:} = GetInstantPose(obj,obj.time_step(i));
                end
                
                obj.path_poses = pose;
                
            elseif isequal(obj.path_type ,'Poses')
                
                obj.time_step = linspace(obj.path_time_range(1),obj.path_time_range(2),size(obj.path_poses,1));
                variable_data = cell2mat(obj.path_poses);
                for i = 1:size(variable_data,2)
                    obj.parametric_functions{i} = fit(obj.time_step',variable_data(:,i),'cubicinterp');
                end
                obj.parametric_functions = obj.parametric_functions';
            end
        end
        
        %% A function to get the pose q
        function [pose] = GetInstantPose(obj,time)
            
            if isequal(obj.path_type ,'Polynomials')
                for i = 1:size(obj.parametric_functions,1)
                    pose(i) = polyval(obj.parametric_functions{i},time);
                end
            elseif isequal(obj.path_type ,'Nonpolynomials')
                pose = double(subs(obj.parametric_functions,time))';
                %                 for i = 1:size(obj.parametric_functions,1)
                %                     pose(i) = double(subs(obj.parametric_functions{i},time));
                %                 end
            elseif isequal(obj.path_type ,'Poses')
                for i = 1:size(obj.parametric_functions,1)
                    pose(i) =  feval(obj.parametric_functions{i},time);
                end
            end
            
        end
        
        %% A function to get the boundary of the time interval for analysis
        function [] = TrajectoryTimeRange(obj,path_type)
            start_times = [];end_times = [];
            time_lower_bound = []; time_upper_bound = [];
            q = obj.parametric_functions;
            q_lower_bound = obj.parametric_functions;
            q_upper_bound = obj.parametric_functions;
            if isequal(path_type ,'Polynomials')
                for i = 1:obj.model.numDofs
                    q_upper_bound{i}(:,end) = q{i}(:,end) - obj.model.bodyModel.q_max(i);
                    time_upper_bound = [time_upper_bound; roots(q_upper_bound{i})];
                end
            elseif isequal(path_type,'Nonpolynomials')
                %                 for i = 1:obj.model.numDofs
                %                     if ~isnumeric(obj.parametric_functions{i})
                %                         q_upper_bound{i}(:,end) = q{i}(:,end) - obj.model.bodyModel.q_max(i);
                %                         time_upper_bound = [time_upper_bound; double(solve(q_upper_bound{i}))];
                %                     end
                %                 end
                Message = ['Path time boundary set to: ',num2str(obj.path_time_range)];
                disp(Message)
            elseif isequal(path_type ,'Poses')
                Message = ['Path time boundary set to: ',num2str(obj.path_time_range)];
                disp(Message)
            else
                CASPR_log.Error('Path type is invalid');
            end
            
            if ~isempty(time_upper_bound)
                time_upper_bound = min(unique(real(time_upper_bound(time_upper_bound>0))));
                obj.path_time_range = [0,time_upper_bound];
            end
        end
        
        %% A function the handle the RREF data from the poses
        function [J_d] = JacobianData(obj)
            % Update the model according to the path, return the last column data of the model.L
            for i = 1:size(obj.path_poses,1)
                obj.model.update(obj.path_poses{i,:}', zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1));
                %                 cable_norm = obj.model.cableLengths';
                %                 L = obj.model.L';
                %                 norm_o_L = diag(cable_norm);
                L{i} = obj.model.L'*diag(obj.model.cableLengths');
            end
            J_d = L;
        end
        
        %% A function to estimate the possible interpolated curve for the RREF data
        function [interpolfit] = PolynomialFitting(obj,Jacobian_Matrics)
            null_data_count = [];
            for i = 1:length(Jacobian_Matrics)
                n_n_matrix = Jacobian_Matrics{i}(:,1:end-1);
                n_1_matrix = Jacobian_Matrics{i}(:,end);
                
                matrix_determinant = det(n_n_matrix);
                
                if round(matrix_determinant,obj.tolerance) ~=0
                    column_vector(i,:) = inv(n_n_matrix)*n_1_matrix*matrix_determinant;
                    det_d(i) =  matrix_determinant;
                else
                    null_data_count = [null_data_count,i];
                    det_d(i) = 0;
                    column_vector(i,:) = zeros(1,size(n_n_matrix,1));
                end
                
            end
            
            tmp_time_step = obj.time_step;
            
            if size(null_data_count,2) ~= size(Jacobian_Matrics,2)
                if isempty(null_data_count)
                    for i = 1:size(column_vector,2)
                        [interpolfit{i},gof(i),~]= fit(obj.time_step',column_vector(:,i),'cubicinterp');
                    end
                    [interpolfit{end+1},gof(i),~]= fit(obj.time_step',det_d','cubicinterp');
                    
                else
                    for delete_step = length(null_data_count):-1:1
                        if all(column_vector(null_data_count(delete_step),:)) ==0
                            column_vector(null_data_count(delete_step),:) = [];
                            tmp_time_step(null_data_count(delete_step)) = [];
                        end
                    end
                    
                    for i = 1:size(column_vector,2)
                        [interpolfit{i},gof(i),~]= fit(tmp_time_step',column_vector(:,i),'cubicinterp');
                    end
                    [interpolfit{end+1},gof(i),~]= fit(obj.time_step',det_d','cubicinterp');
                    
                end
                
                for i = 1:size(gof,2)
                    if round(gof(i).sse,obj.tolerance) ~= 0
                        error('Sample data points are not enough and result in bad fitting. Increase the path_fineness variable');
                    end
                end
            else
                interpolfit = [];
            end
        end
        
        %% A function to handle different DOFs and N+M cables robot
        function [time] = TrajectoryFeasibilityVarification(obj,Jacobian_Matrics)
            record_zero = [];
            interpolated_function = [];
            
            if obj.model.numDofs + 1 < obj.model.numCables
                check_indices = nchoosek(1:obj.model.numCables,obj.model.numDofs+1);
                for i = 1: size(check_indices,1)
                    for j = 1:size(Jacobian_Matrics,2)
                        remain_Jacobian_Matrics{j} = Jacobian_Matrics{j}(:,check_indices(i,:));
                    end
                    
                    interpolated_function = PolynomialFitting(obj,remain_Jacobian_Matrics);
                    
                    if ~isempty(interpolated_function)
                        [time] = WorkspaceBoundaryTime(obj,interpolated_function);
                        obj.feasible_interval = [obj.feasible_interval;time];
                    end
                    
                    record_zero = [];
                    interpolated_function = [];
                end
                
            elseif obj.model.numDofs + 1 == obj.model.numCables
                
                interpolated_function = PolynomialFitting(obj,Jacobian_Matrics);
                
                if ~isempty(interpolated_function)
                    [time] = WorkspaceBoundaryTime(obj,interpolated_function);
                    obj.feasible_interval = [obj.feasible_interval,time];
                end
            else
                error('Input Model Error');
            end
            
            if ~isempty(obj.feasible_interval)
                obj.feasible_interval = sortrows(obj.feasible_interval,1);
                obj.feasible_interval = round(obj.feasible_interval,obj.tolerance);
                current_interval = obj.feasible_interval(1,:);
                
                for i = 1:size(obj.feasible_interval,1)
                    
                    if obj.feasible_interval(i,1) <= current_interval(end,2) && obj.feasible_interval(i,1) >= current_interval(end,1) && obj.feasible_interval(i,2) >= current_interval(end,2)
                        current_interval(end,:) = [current_interval(end,1),obj.feasible_interval(i,2)];
                    elseif obj.feasible_interval(i,1) <= current_interval(end,2) && obj.feasible_interval(i,1) >= current_interval(end,1) && obj.feasible_interval(i,2) <= current_interval(end,2)
                        %
                    elseif obj.feasible_interval(i,1) >= current_interval(2)
                        current_interval = [current_interval;obj.feasible_interval(i,:)];
                    end
                    
                end
                
                obj.feasible_period = current_interval;
                
                q_ans = [];
                for j = 1:size(current_interval,1)
                    for i = 1:size(current_interval,2)
                        q_ans = [q_ans;GetInstantPose(obj,current_interval(j,i))];
                        
                    end
                end
                obj.feasible_range = q_ans;
            end
            if isempty(obj.feasible_range)
                Message = 'No feasible range inside the workspace';
                disp(Message)
            end
        end
        
        %% A function to find and varify the valid time interval
        function [time] = WorkspaceBoundaryTime(obj,fcn)
            
            polynomial_ans = [];
            time = [];
            continue_count = 1;
            time_ans = obj.path_time_range';
            for i = 1:size(fcn,2)
                tmp_ans = fnzeros(fcn{i}.p);
                polynomial_ans = [polynomial_ans,tmp_ans(1,:)];
            end
            
            for i = 1:length(polynomial_ans)
                if isreal(polynomial_ans(i))
                    if real(polynomial_ans(i)) >= obj.path_time_range(1) && real(polynomial_ans(i)) <= obj.path_time_range(2)
                        time_ans = [time_ans;polynomial_ans(i)];
                    end
                end
            end
            
            time_ans =  unique(time_ans,'rows');
            time_ans = sort(time_ans);
            if size(time_ans,1) == 1
                time_ans = obj.path_time_range;
            end
            
            if length(time_ans) >= 2
                
                for k = 1:length(time_ans)-1
                    
                    
                    for h = 1:size(fcn,2)-1
                        mid_point_test(h) = feval(fcn{h},(time_ans(k)+time_ans(k+1))/2) * feval(fcn{end},(time_ans(k)+time_ans(k+1))/2);
                        lower_bound_test(h) =  feval(fcn{h},time_ans(k)) * feval(fcn{end},time_ans(k));
                        upper_bound_test(h) =  feval(fcn{h},time_ans(k+1)) * feval(fcn{end},time_ans(k+1));
                    end
                    
                    if all(round(mid_point_test,obj.tolerance) < 0) && all(round(lower_bound_test,obj.tolerance) <= 0) && all(round(upper_bound_test,5) <= 0)
                        
                        if continue_count == 1
                            time = [time,[time_ans(k),time_ans(k+1)]];
                        else
                            time = [time;[time_ans(k),time_ans(k+1)]];
                            continue_count = 1;
                        end
                    else
                        continue_count = 0;
                    end
                end
            end
        end
        
        %% A function to draw the full path
        function [] = PathAnimation(obj)
            PlotPath(obj);
            
            %             if ~isempty(obj.drawing)
            %                 delete(obj.drawing.cable_graph)
            %                 delete(obj.drawing.end_effector)
            %             end
            
            time_range = obj.path_time_range;
            
            %time_step = linspace(time_range(1),time_range(2),size(obj.path_poses,1));
            time_step = linspace(time_range(1),time_range(2),size(obj.path_poses,1));
            q_pose = cell2mat(obj.path_poses);
            if ~isequal(obj.path_type ,'Poses')
                for i = 1:size(q_pose,2)
                    [obj.estimated_path_polynomials{i},gof(i),~]= fit(time_step',q_pose(:,i),'cubicinterp');
                end
            else
                obj.estimated_path_polynomials = obj.parametric_functions;
            end
            
            if ~isempty(time_range)
                obj.drawing.cable_graph = [];
                xlim([obj.model.bodyModel.q_min(1)-0.5,obj.model.bodyModel.q_max(1)+0.5]);
                ylim([obj.model.bodyModel.q_min(2)-0.5,obj.model.bodyModel.q_max(2)+0.5]);
                zlim([obj.model.bodyModel.q_min(3)-0.5,obj.model.bodyModel.q_max(3)+0.5]);
                hold on;
                grid on;
                q = q_pose;
                for i = 1:length(time_step)
                    if ~isempty(obj.drawing.cable_graph)
                        delete(obj.drawing.cable_graph);
                    end
                    
                    obj.model.update(q(i,:)', zeros(obj.model.numDofs,1), zeros(obj.model.numDofs,1),zeros(obj.model.numDofs,1));
                    
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
                        
                        obj.drawing.cable_graph(j) = line(drawline_x,drawline_y,drawline_z,'Color','black');
                    end
                    if ~isempty(obj.feasible_period)
                        for k = 1:size(obj.feasible_period,1)
                            if time_step(i)<= obj.feasible_period(k,end) && time_step(i)>= obj.feasible_period(k,1)
                                
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
                    else
                        EE_color = [0.7 0.7 0.7];
                    end
                    
                    if all(Z == 0)
                        obj.drawing.end_effector = fill(X,Y,EE_color);
                    else
                        DT = delaunayTriangulation(X',Y',Z');
                        cubepoints = DT.Points;
                        k_c = convexHull(DT);
                        obj.drawing.end_effector= trisurf(k_c,DT.Points(:,1),DT.Points(:,2),DT.Points(:,3),...
                            'FaceColor',EE_color,'EdgeColor','black','FaceAlpha',0.5);%draw the box
                    end
                    
                    pause(0.05)
                    if i ~= length(time_step)
                        %                         for remove_draw = 1:size(drawee,2)
                        %                         delete(drawee{remove_draw});
                        delete(obj.drawing.end_effector);
                        %                         end
                    end
                end
            end
        end
        
        %% A function to dray path
        function [] = PlotPath(obj)
            
            time_range = sort(unique([obj.path_time_range,obj.feasible_period(:)']));
            for i = 1:size(time_range,2)-1
                
                time_step = linspace(time_range(i),time_range(i+1),size(obj.path_poses,1));
                for j = 1:size(time_step,2)
                    q(j,:) = GetInstantPose(obj,time_step(j));
                end
                hold on;
                
                if any(ismember(obj.feasible_period(:),time_range(i))) && any(ismember(obj.feasible_period(:),time_range(i+1)))
                    obj.drawing.path(i) = plot3(q(:,1),q(:,2),q(:,3),'Color','g');
                else
                    obj.drawing.path(i) = plot3(q(:,1),q(:,2),q(:,3),'--','Color',[0.7 0.7 0.7]);
                end
            end
        end
        
    end
end