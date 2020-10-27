% Class to compute whether a ray inside the obstacles from quadtratic and
% quatic degree s
%
% Author        : Paul Cheng
% Created       : 2020
% Description   :

classdef InterferenceFreeRayConditionCableObstacle < WorkspaceRayConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 9;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF;
        
    end
    
    properties (SetAccess = protected)
        % Set constants
        areDofsTranslation;         % Array for the q of the joint (true if translation and false if rotation)
        is_dof_translation
        numDofs;                    % The number of dofs
        numCables;                  % The number of cables
        is_compiled_mode;
        segment_num;
        u_range;
        G_xyz;
        G_coeffs;
        OB_u;
        obstacle;
        surf_ans_ind = [];
        OA_i_hat;
        is_fitting;
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCableObstacle(model, min_ray_lengths, obstacle)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.obstacle = obstacle;
            w.is_compiled_mode = (model.modelMode == ModelModeType.COMPILED);
            w.segment_num = size(model.cableModel.r_OAs,2);
            for surf_ind = 1:obstacle.surfaceNum
                tmp_ind = repmat(1:w.segment_num,obstacle.surfaceDeg(surf_ind),1);
                w.surf_ans_ind = [w.surf_ans_ind;tmp_ind(:)];
            end
            w.is_fitting = 0;
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            % Variable initialisation
            %             tic
            IFI = [];%interference_free_interval for each segment
            intervals = [];
            intervals_count = 1;
            free_variable_index = ws_ray.freeVariableIndex;
            obj.is_dof_translation = obj.areDofsTranslation(free_variable_index);
            
            q_begin = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(1);ws_ray.fixedVariables(free_variable_index:end)];
            q_end = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(2);ws_ray.fixedVariables(free_variable_index:end)];
            %             [q_begin,q_end]
            if obj.is_dof_translation
                obj.u_range = [0 1];
            else
                obj.u_range = tan([q_begin(free_variable_index),q_end(free_variable_index)]/2);
            end
            if obj.obstacle.boundaryNum ~= 0
                [obj.G_coeffs,obj.G_xyz] = obj.GetGxyz(model,q_begin,q_end,free_variable_index,obj.is_dof_translation);
            end
            [obj.OB_u,~]  = GetOB_u(obj,model,q_begin,q_end,free_variable_index,obj.is_dof_translation);
            
            [OA_i_begin,OB_i_begin] = obj.GetSegmentData(model,q_begin);
            [OA_i_end,OB_i_end] = obj.GetSegmentData(model,q_end);
            
            for i = 1:obj.segment_num
                [obj.OA_i_hat(i,:),~] = obj.LineIntersection3D([OB_i_begin(i,:);OB_i_end(i,:)],[OA_i_begin(i,:);OA_i_end(i,:)]);
            end
            
            %             toc
            
            u_obstacle = [];  u_segment_surface_index_1 = [];
            u_Gxyz = [];      u_segment_surface_index_2 = [];
            u_boundary_F = [];      u_segment_surface_index_3 = [];
            u_all = [];             u_segment_surface_index_all = [];
            if ~obj.is_fitting
%                 for bound_ind = 1:obj.obstacle.boundaryNum
%                     [~,~,u_segment_surface_index_3] = obj.SegmentIntersection_Gxyz(bound_ind);
%                     u_Gxyz = [u_Gxyz;u_segment_surface_index_3];
%                     
%                 end
%                 tic
%                 for segment_index = 1:obj.segment_num
%                     
%                     interval_count = 1;
%                     u_surface_index_all = [];
%                     u_segment_surface_index_all = [];
%                     for surf_ind = 1:obj.obstacle.surfaceNum
%                         [~,u_segment_surface_index_1] = obj.Cable_Obstacle_Surface_Intersection_1(segment_index,surf_ind);
%                         if obj.is_dof_translation
%                             
%                             [~,~,u_segment_surface_index_2] = obj.LineSegment_Surface_Intersection_1(segment_index,OB_i_begin(segment_index,:),OB_i_end(segment_index,:),surf_ind,0);
%                         else
%                             [~,u_segment_surface_index_2] = obj.CurveSegment_Surface_Intersection_1(segment_index,surf_ind);
%                         end
%                         u_segment_surface_index_all = [u_segment_surface_index_all;u_segment_surface_index_1;u_segment_surface_index_2];
%                         
%                     end
%                     if ~isempty(u_Gxyz)
%                         u_segment_surface_index_all = [u_segment_surface_index_all;u_Gxyz(find(ismember(u_Gxyz(:,2),segment_index)),:)];
%                     end
%                     if ~isempty(u_segment_surface_index_all)
%                         [~,u_surface_index_all] = obj.SortAnswerWithIndex(u_segment_surface_index_all);
%                     end
%                     %                     u_segment = [u_segment;obj.u_range'];
%                     %                     u_surface_index_all = [obj.u_range(1),u_surface_index_all(1,2);
%                     %                                             u_surface_index_all;
%                     %                                             obj.u_range(2),u_surface_index_all(end,2)];
%                     u_surface_index_all = [obj.u_range(1),{(1:obj.obstacle.surfaceNum)'};
%                         u_surface_index_all;
%                         obj.u_range(2),{(1:obj.obstacle.surfaceNum)'}];
%                     continue_no_intersected = 0;
%                     for i = 1:size(u_surface_index_all,1)- 1
%                         %                         clf
%                         has_intersected = 0;flag = [];
%                         q1 = q_begin;  q2 = q_begin;
%                         if obj.is_dof_translation
%                             q1 = (q_end - q_begin)*u_surface_index_all{i,1} + q_begin;
%                             q2 = (q_end - q_begin)*u_surface_index_all{i+1,1} + q_begin;
%                         else
%                             q1(free_variable_index,:) = 2*atan(u_surface_index_all{i,1});
%                             q2(free_variable_index,:) = 2*atan(u_surface_index_all{i+1,1});
%                         end
%                         intersected_surf = unique([u_surface_index_all{i,2};u_surface_index_all{i+1,2}]);
%                         
%                         [OA_i,OB_i] = obj.GetSegmentData(model,(q1+q2)/2);
%                         %                         [~,~] = draw_robot(model,(q1+q2)/2);
%                         %                         obj.obstacle.plotObstacle
%                         for j = 1:size(intersected_surf,1)
%                             [flag(j),~,~] = obj.LineSegment_Surface_Intersection_1(segment_index,OA_i(segment_index,:),OB_i(segment_index,:),intersected_surf(j),1);
%                             
%                         end
%                         for j = 1:obj.obstacle.surfaceNum
%                             A_direction(j) = sign(round(obj.obstacle.surfaceEqu{j}(OA_i(segment_index,1),OA_i(segment_index,2),OA_i(segment_index,3)),4));
%                             if A_direction(j) == 0
%                                 A_direction(j) = obj.obstacle.surfaceDirection(j);
%                             end
%                             B_direction(j) = sign(round(obj.obstacle.surfaceEqu{j}(OB_i(segment_index,1),OB_i(segment_index,2),OB_i(segment_index,3)),4));
%                             if B_direction(j) == 0
%                                 B_direction(j) = obj.obstacle.surfaceDirection(j);
%                             end
%                         end
%                         if norm(A_direction - obj.obstacle.surfaceDirection) == 0 || norm(B_direction - obj.obstacle.surfaceDirection) == 0
%                             flag = 1;
%                         end
%                         if any(flag)
%                             has_intersected = 1;
%                         else
%                             has_intersected = 0;
%                         end
% %                         if segment_index == 2
% %                             segment_index
% %                             clf
% %                             [~,~] = draw_robot(model,(q1+q2)/2);
% %                             obj.obstacle.plotObstacle
% %                         end
%                         if ~has_intersected
%                             if ~continue_no_intersected
%                                 IFI{segment_index}(interval_count,:) = [u_surface_index_all{i,1},u_surface_index_all{i+1,1}];
%                                 interval_count = interval_count + 1;
%                             else
%                                 IFI{segment_index}(end) = u_surface_index_all{i+1,1};
%                             end
%                             continue_no_intersected = 1;
%                         else
%                             continue_no_intersected = 0;
%                             
%                         end
%                         
%                         
%                         %                         has_intersected = obj.IntervalVerify(model,q_intersected(:,i),q_intersected(:,i+1));
%                         %
%                         %                         if ~has_intersected
%                         %                             if ~isempty(intervals) && intervals(end) == q_intersected(free_variable_index,i)
%                         %                                 intervals(end) = q_intersected(free_variable_index,i+1);
%                         %                             else
%                         %                                 intervals(intervals_count,:) = [q_intersected(free_variable_index,i),q_intersected(free_variable_index,i+1)];
%                         %                                 intervals_count = intervals_count + 1;
%                         %                             end
%                         %                         end
%                     end
%                     
%                 end
%                 
%                 current_interval = sort(reshape(cell2mat(IFI(:,1)),1,[]));
%                 for i = 2:size(IFI,2)
%                     compare_interval = sort(reshape(cell2mat(IFI(:,i)),1,[]));
%                     current_interval = obj.range_intersection(compare_interval,current_interval);
%                 end
%                 
%                 interval(:,1)  = current_interval(1:2:size(current_interval,2));
%                 interval(:,2)  = current_interval(2:2:size(current_interval,2));
%                 toc
                %                 current_interval = reshape(current_interval,[],2)
                %%
%                 tic
                u_segment_surface_index_all = [];
                for surf_ind = 1:obj.obstacle.surfaceNum
                    if  obj.obstacle.surfaceDeg(surf_ind) ~= 1
                        
                        [~,u_segment_surface_index_1] = obj.Cable_Obstacle_Surface_Intersection(surf_ind);
                        
                    end
                    if obj.is_dof_translation
                        
                        [~,~,u_segment_surface_index_2] = obj.LineSegment_Surface_Intersection(OB_i_begin,OB_i_end,surf_ind,0);
                    else
                        [~,u_segment_surface_index_2] = obj.CurveSegment_Surface_Intersection(surf_ind);
                        
                    end
                    % u_all = [u_value;u_obstacle;u_boundary_G];
                    u_segment_surface_index_all = [u_segment_surface_index_all;u_segment_surface_index_1;u_segment_surface_index_2];
                    
                end
                
                for bound_ind = 1:obj.obstacle.boundaryNum
                    [~,~,u_segment_surface_index_3] = obj.SegmentIntersection_Gxyz(bound_ind);
                    u_segment_surface_index_all = [u_segment_surface_index_all;u_segment_surface_index_3];
                    
                end
                %%
                %             toc
                
            else
                
            end
            
            if ~isempty(u_segment_surface_index_all)
                [u_all,u_segment_surface_index_all] = obj.SortAnswerWithIndex(u_segment_surface_index_all);
            end
            
            %%
            %              tic
            u_all = [u_all;obj.u_range'];
            q_intersected = repmat(q_begin,1,size(u_all,1));
            
            if obj.is_dof_translation
                q_intersected = (q_end - q_begin)*u_all' + q_begin;
            else
                q_intersected(free_variable_index,:) = 2*atan(u_all);
            end
            %             q_intersected = [q_intersected,q_begin,q_end];
            q_intersected = unique(round(q_intersected',obj.ROUNDING_DIGIT),'rows')';
            
            for i = 1:size(q_intersected,2) - 1
                
                has_intersected = obj.IntervalVerify(model,q_intersected(:,i),q_intersected(:,i+1));
                
                if ~has_intersected
                    if ~isempty(intervals) && intervals(end) == q_intersected(free_variable_index,i)
                        intervals(end) = q_intersected(free_variable_index,i+1);
                    else
                        intervals(intervals_count,:) = [q_intersected(free_variable_index,i),q_intersected(free_variable_index,i+1)];
                        intervals_count = intervals_count + 1;
                    end
                end
            end
%                         toc
            
        end
        
        %%
        function has_intersected = IntervalVerify(obj,model,q1,q2)
            [OA_i,OB_i] = obj.GetSegmentData(model,(q1+q2)/2);
%             clf
%             obj.obstacle.plotObstacle
%                                                 [~,b] = draw_robot(model,q1+q2)/2);
            %                                     delete(a)
            for surf_ind = 1:obj.obstacle.surfaceNum
                [flag(surf_ind),~,~] = LineSegment_Surface_Intersection(obj,OA_i,OB_i,surf_ind,1);
                if any(flag)
                    has_intersected = 1;
                    %                                                          delete(b)
                    return;
                end
            end
            has_intersected = 0;
            %                                       delete(b)
        end
        
        
        %%
        function [o1,o2] = SortAnswerWithIndex(obj,value)
            o2 = [];o1 = [];
            val = round(value(:,1),obj.ROUNDING_DIGIT);
            [uni_val,ia,~] = unique(val,'rows');
            remove_index = [find(uni_val(:,1) < obj.u_range(1));find(uni_val(:,1) > obj.u_range(2))];
            uni_val(remove_index) = [];
            
            if ~isempty(uni_val)
                for i = 1:size(uni_val,1)
                    index = find(ismember(val,uni_val(i)));
                    tmp_val = {uni_val(i),value(index,3)};
                    o1 = [o1;value(index,1)];
                    o2 = [o2;tmp_val];
                end
            end
        end
        
        %% function to find intersection between a line segment and the obstacle surface
        function [flag,v,seg_surf_ind] = LineSegment_Surface_Intersection(obj,P_begin,P_end,surf_ind,need_varify)
            
            v = []; seg_surf_ind = []; flag = 0;
            t_coeff = [];
            %             tic
            for i = 1:obj.segment_num
                t_coeff = s_coeffs_T(P_end(i,:)',P_begin(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                t_roots = roots(t_coeff);
                imag_index = find(imag(t_roots) ~= 0);
                t_roots(imag_index) = [];
                if ~isempty(t_roots)
                    v = [v;t_roots];
                    seg_surf_ind = [seg_surf_ind;[t_roots, repmat([i,surf_ind],size(t_roots))]];
                end
            end
            %             toc
            
            if ~isempty(seg_surf_ind) && need_varify
                remove_ind = v>1 | v<0;
                seg_surf_ind(remove_ind,:) = [];
                v(remove_ind,:) = [];
                [v,ia,~] = unique(v);
                %                 seg_surf_ind = seg_surf_ind(ia,:);
                %                 v = seg_surf_ind(:,1);
                %                  tic
                for i = 1:size(v,1)
                    
                    v_related_surf_ind = find(ismember(seg_surf_ind(:,1),v(i)));
                    segment_ind = seg_surf_ind(v_related_surf_ind,2);
                    %                     for j = 1:obj.segment_num
                    %                         intersection_pt(j,:) = (P_end(j,:) - P_begin(j,:))*v(i) + P_begin(j,:);
                    %                     end
                    %%
                    for j = 1:size(v_related_surf_ind,1)
                        intersection_pt = (P_end(segment_ind(j),:) - P_begin(segment_ind(j),:))*v(i) + P_begin(segment_ind(j),:);
                        
                        %%
                        %                         dd(j) = scatter3(intersection_pt(:,1),intersection_pt(:,2),intersection_pt(:,3),'filled')
                        in_surf =  intersection_pt(1) >= obj.obstacle.surfaceBoundXYZ{surf_ind}(1) &&...
                            intersection_pt(1) <= obj.obstacle.surfaceBoundXYZ{surf_ind}(2) &&...
                            intersection_pt(2) >= obj.obstacle.surfaceBoundXYZ{surf_ind}(3) &&...
                            intersection_pt(2) <= obj.obstacle.surfaceBoundXYZ{surf_ind}(4) &&...
                            intersection_pt(3) >= obj.obstacle.surfaceBoundXYZ{surf_ind}(5) &&...
                            intersection_pt(3) <= obj.obstacle.surfaceBoundXYZ{surf_ind}(6);
                        
                        if in_surf
                            flag = 1;
                            %                             delete(dd)
                            return;
                        end
                    end
                    %                     for j = 1:obj.obstacle.surfaceNum
                    %                         tmp_in_surf = sign(round(obj.obstacle.surfaceEqu{j}(intersection_pt(:,1),intersection_pt(:,2),intersection_pt(:,3)),4));
                    %                         tmp_in_surf(tmp_in_surf==0) = obj.obstacle.surfaceDirection(j);
                    %                         in_surf(:,j) = tmp_in_surf;
                    %                     end
                    %                     delete(dd)
                    %                     for j = 1:obj.segment_num
                    %                         if isequal(in_surf(j,:),obj.obstacle.surfaceDirection)
                    %                             flag = 1;
                    %                             return;
                    %                         end
                    %                     end
                    
                end
                %                 toc
                flag = 0;
            end
        end
        
        %% function to find intersection between a line segment and the obstacle surface
        function [flag,v,seg_surf_ind] = LineSegment_Surface_Intersection_1(obj,segment_index,P_begin,P_end,surf_ind,need_varify)
            
            v = []; seg_surf_ind = []; flag = 0;
            t_coeff = [];
            %             tic
            %             for i = 1:obj.segment_num
            t_coeff = s_coeffs_T(P_end',P_begin',obj.obstacle.surfaceCoeffs{surf_ind});
            t_roots = roots(t_coeff);
            imag_index = find(imag(t_roots) ~= 0);
            t_roots(imag_index) = [];
            if ~isempty(t_roots)
                v = [v;t_roots];
                seg_surf_ind = [seg_surf_ind;[t_roots, repmat([segment_index,surf_ind],size(t_roots))]];
            end
            %             end
            %             toc
            
            if ~isempty(seg_surf_ind) && need_varify
                if ~isempty(obj.obstacle.surf2boundary)
                    related_boundary_index = obj.obstacle.surf2boundary(find(ismember(obj.obstacle.surf2boundary(:,1),surf_ind)),2:end);
                    tmp_index = obj.obstacle.boundary2surf(find(ismember(obj.obstacle.boundary2surf(:,1),related_boundary_index)),2:end);
                    related_surfaces_index = unique(tmp_index(:));
                    surf_ind = related_surfaces_index;
                end
                remove_ind = v>1 | v<0;
                seg_surf_ind(remove_ind,:) = [];
                v(remove_ind,:) = [];
                [v,ia,~] = unique(v);
                %                 seg_surf_ind = seg_surf_ind(ia,:);
                %                 v = seg_surf_ind(:,1);
                %                  tic
                for i = 1:size(v,1)
                    
                    v_related_surf_ind = find(ismember(seg_surf_ind(:,1),v(i)));
                    segment_ind = seg_surf_ind(v_related_surf_ind,2);
                    
                    for j = 1:size(v_related_surf_ind,1)
                        intersection_pt = (P_end - P_begin)*v(i) + P_begin;
                        
                        %%
                        
                        %                         dd = scatter3(intersection_pt(:,1),intersection_pt(:,2),intersection_pt(:,3),'filled')
                        %                         in_surf =  intersection_pt(1) >= obj.obstacle.surfaceBoundXYZ{surf_ind}(1) &&...
                        %                             intersection_pt(1) <= obj.obstacle.surfaceBoundXYZ{surf_ind}(2) &&...
                        %                             intersection_pt(2) >= obj.obstacle.surfaceBoundXYZ{surf_ind}(3) &&...
                        %                             intersection_pt(2) <= obj.obstacle.surfaceBoundXYZ{surf_ind}(4) &&...
                        %                             intersection_pt(3) >= obj.obstacle.surfaceBoundXYZ{surf_ind}(5) &&...
                        %                             intersection_pt(3) <= obj.obstacle.surfaceBoundXYZ{surf_ind}(6);
                        for k = 1:size(surf_ind,1)
                            in_surf(k) = sign(round(obj.obstacle.surfaceEqu{surf_ind(k)}(intersection_pt(1),intersection_pt(2),intersection_pt(3)),4));
                            %                         delete(dd)
                            if in_surf(k) == 0
                                in_surf(k) =  obj.obstacle.surfaceDirection(surf_ind(k));
                            end
                        end
                        if all(in_surf == obj.obstacle.surfaceDirection(surf_ind))
                            flag = 1;
                            %                             delete(dd)
                            return;
                        else
                            flag = 0;
                        end
                    end
                    %                     for j = 1:obj.obstacle.surfaceNum
                    %                         tmp_in_surf = sign(round(obj.obstacle.surfaceEqu{j}(intersection_pt(:,1),intersection_pt(:,2),intersection_pt(:,3)),4));
                    %                         tmp_in_surf(tmp_in_surf==0) = obj.obstacle.surfaceDirection(j);
                    %                         in_surf(:,j) = tmp_in_surf;
                    %                     end
                    %                     delete(dd)
                    %                     for j = 1:obj.segment_num
                    %                         if isequal(in_surf(j,:),obj.obstacle.surfaceDirection)
                    %                             flag = 1;
                    %                             return;
                    %                         end
                    %                     end
                    
                end
                %                 toc
                flag = 0;
            end
        end
        %% function to find intersection between boundary of implicit surface to cable surface
        function [u_edge,v_edge,val_seg_surf] = SegmentIntersection_Gxyz(obj,bound_ind)
            u_edge = []; segment_ind = []; v_edge= [];
            related_surface_index = obj.obstacle.boundary2surf(find(ismember(obj.obstacle.boundary2surf(:,1),bound_ind)),2:end);
            P_j = [];v_value = [];val_seg_surf = [];
            u_value = [];
            v_roots = []; v = []; segment_ind = [];
            %             obj.obstacle.plotObstacle;
            %             hold on
            for i = 1:obj.segment_num
                
                v_coeffs = g_coeffs(obj.obstacle.boundaryEquCoeff{bound_ind},obj.G_coeffs(i,:)');
                
                v_roots = roots(v_coeffs);
                imag_index = find(imag(v_roots) ~= 0);
                v_roots(imag_index) = [];
                if ~isempty(v_roots)
                    v_roots(v_roots > 1) = [];v_roots(v_roots < 0) = [];
                    
                    if ~isempty(v_roots)
                        %                         ha(i)= fimplicit3(obj.G_xyz{i},[0 1 0 1 0 1],'FaceColor',[0.2,0.2,0.2],'FaceAlpha',0.8,'MeshDensity',35,'EdgeColor','none');
                        %                 delete(ha)
                        v = [v;v_roots];
                        for j = 1:size(v_roots,1)
                            P_j = [P_j,obj.obstacle.boundaryEqu{bound_ind}(v_roots(j))];
                        end
                        segment_ind = [segment_ind;[v_roots, repmat([i,bound_ind],size(v_roots))]];
                    end
                end
            end
            for i = 1:size(v,1)
                [u_val,v_val] = obj.Finduv(obj.OB_u{segment_ind(i,2)},obj.OA_i_hat(segment_ind(i,2),:),P_j(:,i));
                if ~isempty(u_val) && u_val <= obj.u_range(2) &&  u_val >= obj.u_range(1) &&...
                        v_val <= 1 &&  v_val >= 0
                    u_edge = [u_edge;u_val];
                    v_edge = [v_edge;v_val];
                    %                     val_seg_surf = [u_val,segment_ind(i,2:end)];
                    val_seg_surf = [repmat(u_val,size(related_surface_index))',repmat(segment_ind(i,2),size(related_surface_index))',related_surface_index'];
                end
            end
            
        end
        
        %% function to find intersection between OB_u and obstacle surface
        function [u_edge,seg_surf_ind] = CurveSegment_Surface_Intersection(obj,surf_ind)
            seg_surf_ind = [];
            u_edge = [];
            
            for i = 1:obj.segment_num
                u_coeff = s_coeffs_O(obj.OB_u{i},obj.obstacle.surfaceCoeffs{surf_ind});
                u_roots = roots(u_coeff);
                imag_index = find(imag(u_roots) ~= 0);
                u_roots(imag_index) = [];
                if ~isempty(u_roots)
                    u_edge = [u_edge;u_roots];
                    seg_surf_ind = [seg_surf_ind;[u_roots, repmat([i,surf_ind],size(u_roots))]];
                end
            end
        end
        
        %% function to calculate the u,v where points on the G(u,v)
        function [u,v] = Finduv(obj,OB_u,OA,P)
            if obj.is_dof_translation
                a1 = OB_u(1,1); b1 = OB_u(1,2);  d1 = OA(1);
                a2 = OB_u(2,1); b2 = OB_u(2,2);  d2 = OA(2);
                % a3 = OB_u(3,1); b3 = OB_u(3,2); d3 = OA(3);
                x = P(1); y = P(2); z = P(3);
                u = (d2 - b2 + ((b1 - d1)*(d2 - y))/(d1 - x))/(a2 - (a1*(d2 - y))/(d1 - x));
                if isnan(u)
                    u = [];v = [];
                    return;
                end
                v = -(d1 - x)/(b1 - d1 + a1*u);
                
            else
                
                a1 = OB_u(1,1); b1 = OB_u(1,2); c1 = OB_u(1,3);  d1 = OA(1);
                a2 = OB_u(2,1); b2 = OB_u(2,2); c2 = OB_u(2,3);  d2 = OA(2);
                %             a3 = OB_u(3,1); b3 = OB_u(3,2); c3 = OB_u(3,3);  d3 = OA(3);
                x = P(1); y = P(2); z = P(3);
                u = [-(b1*d2 - b2*d1 + b2*x - b1*y + (b1^2*d2^2 + b2^2*d1^2 + b2^2*x^2 + b1^2*y^2 - 4*d2^2*x^2 - 4*d1^2*y^2 - 4*a1*c1*d2^2 - 4*a2*c2*d1^2 - 4*a2*c2*x^2 - 4*a1*c1*y^2 + 4*a1*d2^2*x + 4*a2*d2*x^2 + 4*a1*d1*y^2 + 4*a2*d1^2*y - 2*b2^2*d1*x + 4*c1*d2^2*x - 2*b1^2*d2*y + 4*c2*d2*x^2 + 4*c1*d1*y^2 + 4*c2*d1^2*y + 8*a1*c1*d2*y - 4*a1*c2*d1*y - 4*a2*c1*d1*y + 2*b1*b2*d1*y - 4*a2*d1*d2*x - 4*a1*d1*d2*y - 4*c2*d1*d2*x - 4*c1*d1*d2*y + 4*a1*c2*x*y + 4*a2*c1*x*y - 2*b1*b2*x*y - 4*a1*d2*x*y - 4*a2*d1*x*y - 4*c1*d2*x*y - 4*c2*d1*x*y + 8*d1*d2*x*y + 4*a1*c2*d1*d2 + 4*a2*c1*d1*d2 - 2*b1*b2*d1*d2 - 4*a1*c2*d2*x - 4*a2*c1*d2*x + 8*a2*c2*d1*x + 2*b1*b2*d2*x)^(1/2))/(2*(a1*d2 - a2*d1 + a2*x - a1*y - d2*x + d1*y));
                    (b2*d1 - b1*d2 - b2*x + b1*y + (b1^2*d2^2 + b2^2*d1^2 + b2^2*x^2 + b1^2*y^2 - 4*d2^2*x^2 - 4*d1^2*y^2 - 4*a1*c1*d2^2 - 4*a2*c2*d1^2 - 4*a2*c2*x^2 - 4*a1*c1*y^2 + 4*a1*d2^2*x + 4*a2*d2*x^2 + 4*a1*d1*y^2 + 4*a2*d1^2*y - 2*b2^2*d1*x + 4*c1*d2^2*x - 2*b1^2*d2*y + 4*c2*d2*x^2 + 4*c1*d1*y^2 + 4*c2*d1^2*y + 8*a1*c1*d2*y - 4*a1*c2*d1*y - 4*a2*c1*d1*y + 2*b1*b2*d1*y - 4*a2*d1*d2*x - 4*a1*d1*d2*y - 4*c2*d1*d2*x - 4*c1*d1*d2*y + 4*a1*c2*x*y + 4*a2*c1*x*y - 2*b1*b2*x*y - 4*a1*d2*x*y - 4*a2*d1*x*y - 4*c1*d2*x*y - 4*c2*d1*x*y + 8*d1*d2*x*y + 4*a1*c2*d1*d2 + 4*a2*c1*d1*d2 - 2*b1*b2*d1*d2 - 4*a1*c2*d2*x - 4*a2*c1*d2*x + 8*a2*c2*d1*x + 2*b1*b2*d2*x)^(1/2))/(2*(a1*d2 - a2*d1 + a2*x - a1*y - d2*x + d1*y))];
                %             u2 = [-(b1*d3 - b3*d1 + b3*x - b1*z + (b1^2*d3^2 + b3^2*d1^2 + b3^2*x^2 + b1^2*z^2 - 4*d3^2*x^2 - 4*d1^2*z^2 - 4*a1*c1*d3^2 - 4*a3*c3*d1^2 - 4*a3*c3*x^2 + 4*a1*d3^2*x + 4*a3*d3*x^2 - 4*a1*c1*z^2 - 2*b3^2*d1*x + 4*a1*d1*z^2 + 4*a3*d1^2*z + 4*c1*d3^2*x + 4*c3*d3*x^2 - 2*b1^2*d3*z + 4*c1*d1*z^2 + 4*c3*d1^2*z - 4*a3*d1*d3*x + 8*a1*c1*d3*z - 4*a1*c3*d1*z - 4*a3*c1*d1*z + 2*b1*b3*d1*z - 4*a1*d1*d3*z - 4*c3*d1*d3*x - 4*c1*d1*d3*z + 4*a1*c3*x*z + 4*a3*c1*x*z - 2*b1*b3*x*z - 4*a1*d3*x*z - 4*a3*d1*x*z - 4*c1*d3*x*z - 4*c3*d1*x*z + 8*d1*d3*x*z + 4*a1*c3*d1*d3 + 4*a3*c1*d1*d3 - 2*b1*b3*d1*d3 - 4*a1*c3*d3*x - 4*a3*c1*d3*x + 8*a3*c3*d1*x + 2*b1*b3*d3*x)^(1/2))/(2*(a1*d3 - a3*d1 + a3*x - a1*z - d3*x + d1*z));
                %                 (b3*d1 - b1*d3 - b3*x + b1*z + (b1^2*d3^2 + b3^2*d1^2 + b3^2*x^2 + b1^2*z^2 - 4*d3^2*x^2 - 4*d1^2*z^2 - 4*a1*c1*d3^2 - 4*a3*c3*d1^2 - 4*a3*c3*x^2 + 4*a1*d3^2*x + 4*a3*d3*x^2 - 4*a1*c1*z^2 - 2*b3^2*d1*x + 4*a1*d1*z^2 + 4*a3*d1^2*z + 4*c1*d3^2*x + 4*c3*d3*x^2 - 2*b1^2*d3*z + 4*c1*d1*z^2 + 4*c3*d1^2*z - 4*a3*d1*d3*x + 8*a1*c1*d3*z - 4*a1*c3*d1*z - 4*a3*c1*d1*z + 2*b1*b3*d1*z - 4*a1*d1*d3*z - 4*c3*d1*d3*x - 4*c1*d1*d3*z + 4*a1*c3*x*z + 4*a3*c1*x*z - 2*b1*b3*x*z - 4*a1*d3*x*z - 4*a3*d1*x*z - 4*c1*d3*x*z - 4*c3*d1*x*z + 8*d1*d3*x*z + 4*a1*c3*d1*d3 + 4*a3*c1*d1*d3 - 2*b1*b3*d1*d3 - 4*a1*c3*d3*x - 4*a3*c1*d3*x + 8*a3*c3*d1*x + 2*b1*b3*d3*x)^(1/2))/(2*(a1*d3 - a3*d1 + a3*x - a1*z - d3*x + d1*z))];
                v = -((u.^2 + 1)*(d1 - x))./(c1 - d1 + b1.*u + a1*u.^2 - d1*u.^2);
                OB = [polyval(OB_u(1,:),u),...
                    polyval(OB_u(2,:),u),...
                    polyval(OB_u(3,:),u)]./(1+u.^2);
                m1 = [OB - OA]'./vecnorm([OB - OA]');
                m2 = (P-OA')/ norm(P-OA');
                cor_ind = all(round(m1 - m2,obj.ROUNDING_DIGIT)==0);
                
                %
                u = u(cor_ind);
                v = v(cor_ind);
            end
        end
        
        %% %function to find intersection between obstacle and cable swept surface
        function [u_obs,seg_surf_ind] = Cable_Obstacle_Surface_Intersection(obj,surf_ind)
            u_coeff = [];
            u_obs = [];
            seg_surf_ind = [];
            if obj.obstacle.surfaceDeg(surf_ind) == 1
                return;
            end
            for i = 1:obj.segment_num
                if ~obj.is_dof_translation
                    if obj.obstacle.surfaceDeg(surf_ind) == 4
                        a = H4_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        b = H3_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        c = H2_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        d = H1_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        e = H0_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        u_coeff = [u_coeff;u_coeffs_4_deg(a,b,c,d,e)];
                    elseif obj.obstacle.surfaceDeg(surf_ind) == 3
                        a = H3_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        b = H2_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        c = H1_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        d = H0_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        u_coeff = [u_coeff;u_coeffs_3_deg(a,b,c,d)];
                    elseif obj.obstacle.surfaceDeg(surf_ind) == 2
                        a = H2_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        b = H1_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        c = H0_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        u_coeff = [u_coeff;u_coeffs_2_deg(a,b,c)];
                    end
                else
                    if obj.obstacle.surfaceDeg(surf_ind) == 4
                        a = zeros(1,9); b = zeros(1,7); c = zeros(1,5); d = zeros(1,3); e = zeros(1,1);
                        a(5:end) = H4_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        b(4:end) = H3_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        c(3:end) = H2_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        d(2:end) = H1_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        e(1:end) = H0_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        u_coeff = [u_coeff;u_coeffs_4_deg(a,b,c,d,e)];
                    elseif obj.obstacle.surfaceDeg(surf_ind) == 3
                        a = zeros(1,7); b = zeros(1,5); c = zeros(1,3); d = zeros(1,1);
                        a(4:end) = H3_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        b(3:end) = H2_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        c(2:end) = H1_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        d(1:end) = H0_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        u_coeff = [u_coeff;u_coeffs_3_deg(a,b,c,d)];
                    elseif obj.obstacle.surfaceDeg(surf_ind) == 2
                        a = zeros(1,5); b = zeros(1,3); c = zeros(1,1);
                        a(3:end) = H2_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        b(2:end) = H1_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        c = H0_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                        u_coeff = [u_coeff;u_coeffs_2_deg(a,b,c)];
                    end
                end
                
            end
            u_coeff = round(u_coeff,obj.ROUNDING_DIGIT);
            for i = 1:size(u_coeff,1)
                if any(isnan(u_coeff(i,:)))
                    u_coeff(i,:) = zeros(size(u_coeff(i,:)));
                end
                u_roots = roots(u_coeff(i,:));
                imag_index = find(imag(u_roots) ~= 0);
                u_roots(imag_index) = [];
                if ~isempty(u_roots)
                    wrong_u_index = [];
                    for k = 1:size(u_roots,1)
                        if obj.is_dof_translation
                            v_coeff = v_coeffs_T(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind},u_roots(k));
                        else
                            v_coeff = v_coeffs_O(obj.OB_u{i},obj.OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind},u_roots(k));
                        end
                        %                         v_roots  = roots(round(v_coeff,obj.ROUNDING_DIGIT));
                        v_roots  = round(roots(v_coeff),3);
                        imag_index = find(imag(v_roots) ~= 0);
                        v_roots(imag_index) = [];
                        
                        if isempty(v_roots)
                            wrong_u_index = [wrong_u_index;k];
                        else
                            v_roots = v_roots(1);
                            if v_roots > 1 || v_roots < 0
                                wrong_u_index = [wrong_u_index;k];
                            end
                        end
                    end
                    u_roots(wrong_u_index) = [];
                    if ~isempty(u_roots)
                        u_obs = [u_obs;u_roots];
                        seg_surf_ind = [seg_surf_ind;[u_roots, repmat([i,surf_ind],size(u_roots))]];
                    end
                end
            end
            
        end
        
        %% %function to find intersection between obstacle and cable swept surface
        function [u_obs,seg_surf_ind] = Cable_Obstacle_Surface_Intersection_1(obj,segment_index,surf_ind)
            u_coeff = [];
            u_obs = [];
            seg_surf_ind = [];
            if obj.obstacle.surfaceDeg(surf_ind) == 1
                return;
            end
            %             for i = 1:obj.segment_num
            if ~obj.is_dof_translation
                if obj.obstacle.surfaceDeg(surf_ind) == 4
                    a = H4_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b = H3_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c = H2_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    d = H1_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    e = H0_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff = [u_coeff;u_coeffs_4_deg(a,b,c,d,e)];
                elseif obj.obstacle.surfaceDeg(surf_ind) == 3
                    a = H3_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b = H2_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c = H1_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    d = H0_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff = [u_coeff;u_coeffs_3_deg(a,b,c,d)];
                elseif obj.obstacle.surfaceDeg(surf_ind) == 2
                    a = H2_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b = H1_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c = H0_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff = [u_coeff;u_coeffs_2_deg(a,b,c)];
                end
            else
                if obj.obstacle.surfaceDeg(surf_ind) == 4
                    a = zeros(1,9); b = zeros(1,7); c = zeros(1,5); d = zeros(1,3); e = zeros(1,1);
                    a(5:end) = H4_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b(4:end) = H3_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c(3:end) = H2_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    d(2:end) = H1_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    e(1:end) = H0_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff = [u_coeff;u_coeffs_4_deg(a,b,c,d,e)];
                elseif obj.obstacle.surfaceDeg(surf_ind) == 3
                    a = zeros(1,7); b = zeros(1,5); c = zeros(1,3); d = zeros(1,1);
                    a(4:end) = H3_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b(3:end) = H2_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c(2:end) = H1_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    d(1:end) = H0_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff = [u_coeff;u_coeffs_3_deg(a,b,c,d)];
                elseif obj.obstacle.surfaceDeg(surf_ind) == 2
                    a = zeros(1,5); b = zeros(1,3); c = zeros(1,1);
                    a(3:end) = H2_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b(2:end) = H1_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c = H0_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff = [u_coeff;u_coeffs_2_deg(a,b,c)];
                end
            end
            
            %             end
            
            u_coeff = round(u_coeff,obj.ROUNDING_DIGIT);
            if any(isnan(u_coeff))
                return;
            end
            u_roots = roots(u_coeff);
            imag_index = find(imag(u_roots) ~= 0);
            u_roots(imag_index) = [];
            
            if ~isempty(u_roots)
                wrong_u_index = [];
                for k = 1:size(u_roots,1)
                    if obj.is_dof_translation
                        v_coeff = v_coeffs_T(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind},u_roots(k));
                    else
                        v_coeff = v_coeffs_O(obj.OB_u{segment_index},obj.OA_i_hat(segment_index,:)',obj.obstacle.surfaceCoeffs{surf_ind},u_roots(k));
                    end
                    %                         v_roots  = roots(round(v_coeff,obj.ROUNDING_DIGIT));
                    v_roots  = round(roots(v_coeff),4);
                    imag_index = find(imag(v_roots) ~= 0);
                    v_roots(imag_index) = [];
                    
                    if isempty(v_roots)
                        wrong_u_index = [wrong_u_index;k];
                    else
                        v_roots = v_roots(1);
                        if v_roots > 1 || v_roots < 0
                            wrong_u_index = [wrong_u_index;k];
                        end
                    end
                end
                u_roots(wrong_u_index) = [];
                if ~isempty(u_roots)
                    u_obs = [u_obs;u_roots];
                    seg_surf_ind = [seg_surf_ind;[u_roots, repmat([segment_index,surf_ind],size(u_roots))]];
                end
            end
            
        end
        %%
        function [cable_implicit_coeff,cable_implicit_fun] = GetGxyz(obj,model,q_begin,q_end,free_variable_index,is_dof_translation)
            cable_implicit_fun = []; cable_implicit_coeff = [];
            if is_dof_translation
                sample_size = 2;
            else
                sample_size = 7;
            end
            
            q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
            q_update = q_begin;
            surface_data = {};
            for segment_index = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(segment_index);
                [OA_i_u,OB_i_u]  = obj.GetSegmentData(model,q_update);
                t = [0.1;0.25;0.5;0.75;0.9];
                for j = 1:size(t,1)
                    tmp_data = (OB_i_u - OA_i_u).*t(j) + OA_i_u;
                    for k = 1:obj.segment_num
                        if segment_index == 1 && j == 1
                            surface_data{k}(j,:) = tmp_data(k,:);
                        else
                            surface_data{k}(end+1,:) = tmp_data(k,:);
                        end
                    end
                end
            end
            v = zeros(obj.segment_num,10);
            for segment_index = 1:obj.segment_num
                if is_dof_translation
                    v(segment_index,7:end) = obj.Plane_fit(surface_data{segment_index});
                    cable_implicit_fun{segment_index} =@(x,y,z) v(segment_index,7).*x +  v(segment_index,8).*y +  v(segment_index,9).*z +  v(segment_index,10);
                else
                    [v(segment_index,:),center(segment_index,:)] = obj.Cone_fit(surface_data{segment_index});
                    cable_implicit_fun{segment_index} =@(x,y,z) v(segment_index,1).*x.^2 + v(segment_index,2).*y.^2 + v(segment_index,3).*z.^2 + ...
                        v(segment_index,4).*x.*y + v(segment_index,5).*x.*z + v(segment_index,6).*y.*z + v(segment_index,7).*x + v(segment_index,8).*y +...
                        v(segment_index,9).*z + v(segment_index,10);
                end
            end
            
            cable_implicit_coeff = v;
        end
        %% function to fit the cable swept surface into implicit form for orientation
        function [ v,center ] = Cone_fit(~, surface_data )
            % * center    -  ellispoid or other conic center coordinates [xc; yc; zc]
            % * radii     -  ellipsoid or other conic radii [a; b; c]
            % * evecs     -  the radii directions as columns of the 3x3 matrix
            % * v         -  the 10 parameters describing the ellipsoid / conic algebraically:
            %             %                Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
            %         now       Ax^2 + By^2 + Cz^2 + Dxy + Exz + Fyz + Gx + Hy + Iz + J = 0
            % Dv = d2 - > D'Dv = D'd2 -> v = (D'D)\(D'd2)
            x = surface_data( :, 1 );
            y = surface_data( :, 2 );
            z = surface_data( :, 3 );
            D = [ x .* x + y .* y - 2 * z .* z, ...
                x .* x + z .* z - 2 * y .* y, ...
                2 * x .* y, ...
                2 * x .* z, ...
                2 * y .* z, ...
                2 * x, ...
                2 * y, ...
                2 * z, ...
                1 + 0 * x ];
            % solve the normal system of equations
            d2 = x .* x + y .* y + z .* z; % the RHS of the llsq problem (y's)
            u = ( D' * D ) \ ( D' * d2 );  % solution to the normal equations
            v(1) = u(1) +     u(2) - 1;
            v(2) = u(1) - 2 * u(2) - 1;
            v(3) = u(2) - 2 * u(1) - 1;
            v( 4 : 10 ) = u( 3 : 9 );
            v = v';
            
            % form the algebraic form of the ellipsoid
            A = [ v(1) v(4) v(5) v(7); ...
                v(4) v(2) v(6) v(8); ...
                v(5) v(6) v(3) v(9); ...
                v(7) v(8) v(9) v(10) ];
            % find the center of the ellipsoid
            center = -A( 1:3, 1:3 ) \ v( 7:9 );
            if abs( v(end) ) > 1e-6
                v = -v / v(end); % normalize to the more conventional form with constant term = -1
            else
                v = -sign( v(end) ) * v;
            end
            v(4:9) = 2*v(4:9);
        end
        
        %% function to fit the cable swept surface into implicit form for translation
        function v = Plane_fit(~,surface_data)
            % v(1) -> x  v(2) -> y v(3) -> z v(4) -> d
            vec_1 = surface_data(2,:) - surface_data(1,:);
            vec_2 = surface_data(end,:) - surface_data(1,:);
            %             v(1:3) = cross(vec_1,vec_2);
            v(1) = vec_1(2)*vec_2(3) - vec_1(3)*vec_2(2);
            v(2) = vec_1(3)*vec_2(1) - vec_1(1)*vec_2(3);
            v(3) = vec_1(1)*vec_2(2) - vec_1(2)*vec_2(1);
            v(4) = -v(1:3)*surface_data(1,:)';
            %             A =  [surface_data(:,1:2)  ones(size(surface_data,1),1)];
            %             B = surface_data(:,3);
            % %             coeff = (A'*A) \(A'*B);
            %             coeff = A\B;
            %             v(1) = coeff(1); v(2) = coeff(2);  v(3) = -1; v(4) = coeff(3);
        end
        
        %% functin to get the cable segment data
        function [OA_i,OB_i] = GetSegmentData(obj,model,q)
            q_zero = zeros(model.numDofs,1);
            if obj.is_compiled_mode
                rOAi = model.cableModel.compiled_r_OAs_fn(q,q_zero,q_zero,q_zero);
                OA_i =  rOAi(1:3,:)';
                OB_i = rOAi(4:6,:)';
            else
                model.update(q,q_zero,q_zero,q_zero);
                rOAi = model.cableModel.r_OAs;
                OA_i =  rOAi(1:3,:)';
                OB_i = rOAi(4:6,:)';
            end
        end
        %%
        function [OB_u,OB_u_denominator]  = GetOB_u(obj,model,q_begin,q_end,free_variable_index,is_dof_translation)
            OB_u_denominator = [];
            if is_dof_translation
                sample_size = 2;
                q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
                u_sample = linspace(0,1,sample_size);
                H_i_u_denominator = ones(size(u_sample));
                %                 OB_u_denominator = 1;
                OB_u_deg = 1;
            else
                sample_size = 3;
                q_sample = linspace(1.05*q_begin(free_variable_index),1.05*q_end(free_variable_index),sample_size);
                u_sample = tan(q_sample/2);
                H_i_u_denominator = (1+u_sample.^2);
                %                 OB_u_denominator = @(u) 1+u.^2;
                OB_u_deg = 2;
            end
            q_update = q_begin;
            for segment_index = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(segment_index);
                [~,OB_i_u]  = obj.GetSegmentData(model,q_update);
                for j = 1:obj.segment_num
                    OB_i_u_sample{j}(segment_index,:) = OB_i_u(j,:).*H_i_u_denominator(segment_index);
                end
            end
            for segment_index = 1:obj.segment_num
                for j = 1:3
                    OB_u{segment_index}(j,:) = GeneralMathOperations.PolynomialFit(u_sample', OB_i_u_sample{segment_index}(:,j), OB_u_deg)';
                end
            end
            %OB_u -> [x(u);y(u);z(u)]
        end
        
        %% function to find 2 line intersection point in 3D
        function [P_intersect,distances] = LineIntersection3D(~,PA,PB)
            % Find intersection point of lines in 3D space, in the least squares sense.
            % PA :          Nx3-matrix containing starting point of N lines
            % PB :          Nx3-matrix containing end point of N lines
            % P_Intersect : Best intersection point of the N lines, in least squares sense.
            % distances   : Distances from intersection point to the input lines
            % Anders Eikenes, 2012
            Si = PB - PA; %N lines described as vectors
            ni = Si ./ (sqrt(sum(Si.^2,2))*ones(1,3)); %Normalize vectors
            nx = ni(:,1); ny = ni(:,2); nz = ni(:,3);
            SXX = sum(nx.^2-1);
            SYY = sum(ny.^2-1);
            SZZ = sum(nz.^2-1);
            SXY = sum(nx.*ny);
            SXZ = sum(nx.*nz);
            SYZ = sum(ny.*nz);
            S = [SXX SXY SXZ;SXY SYY SYZ;SXZ SYZ SZZ];
            CX  = sum(PA(:,1).*(nx.^2-1) + PA(:,2).*(nx.*ny)  + PA(:,3).*(nx.*nz));
            CY  = sum(PA(:,1).*(nx.*ny)  + PA(:,2).*(ny.^2-1) + PA(:,3).*(ny.*nz));
            CZ  = sum(PA(:,1).*(nx.*nz)  + PA(:,2).*(ny.*nz)  + PA(:,3).*(nz.^2-1));
            C   = [CX;CY;CZ];
            P_intersect = (S\C)';
            if nargout>1
                N = size(PA,1);
                distances=zeros(N,1);
                for segment_index=1:N %This is faster:
                    ui=(P_intersect-PA(segment_index,:))*Si(segment_index,:)'/(Si(segment_index,:)*Si(segment_index,:)');
                    distances(segment_index)=norm(P_intersect-PA(segment_index,:)-ui*Si(segment_index,:));
                end
                %for segment_index=1:N %http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html:
                %    distances(segment_index) = norm(cross(P_intersect-PA(segment_index,:),P_intersect-PB(segment_index,:))) / norm(Si(segment_index,:));
                %end
            end
        end
        
        %% function to find the interval intersection
        function out=range_intersection(~,first,second)
            % Purpose: Range/interval intersection
            %
            % A and B two ranges of closed intervals written
            % as vectors [lowerbound1 upperbound1 lowerbound2 upperbound2]
            % or as matrix [lowerbound1, lowerbound2, lowerboundn;
            %               upperbound1, upperbound2, upperboundn]
            % A and B have to be sorted in ascending order
            %
            % out is the mathematical intersection A n B
            %
            %
            % EXAMPLE USAGE:
            %   >> out=range_intersection([1 3 5 9],[2 9])
            %   	out =  [2 3 5 9]
            %   >> out=range_intersection([40 44 55 58], [42 49 50 52])
            %   	out =  [42 44]
            %
            % Author: Xavier Beudaert <xavier.beudaert@gmail.com>
            % Original: 10-June-2011
            % Major modification and bug fixing 30-May-2012
            
            % Allocate, as we don't know yet the size, we assume the largest case
            out1(1:(numel(second)+(numel(first)-2)))=0;
            
            k=1;
            while isempty(first)==0 && isempty(second)==0
                % make sure that first is ahead second
                if first(1)>second(1)
                    temp=second;
                    second=first;
                    first=temp;
                end
                
                if first(2)<second(1)
                    first=first(3:end);
                    continue;
                elseif first(2)==second(1)
                    out1(k)=second(1);
                    out1(k+1)=second(1);
                    k=k+2;
                    
                    first=first(3:end);
                    continue;
                else
                    if first(2)==second(2)
                        out1(k)=second(1);
                        out1(k+1)=second(2);
                        k=k+2;
                        
                        first=first(3:end);
                        second=second(3:end);
                        
                    elseif first(2)<second(2)
                        out1(k)=second(1);
                        out1(k+1)=first(2);
                        k=k+2;
                        
                        first=first(3:end);
                    else
                        out1(k)=second(1);
                        out1(k+1)=second(2);
                        k=k+2;
                        
                        second=second(3:end);
                    end
                end
            end
            
            % Remove the tails
            out=out1(1:k-1);
        end
    end
end
