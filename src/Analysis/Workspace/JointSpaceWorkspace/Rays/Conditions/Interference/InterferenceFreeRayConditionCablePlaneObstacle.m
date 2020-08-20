% Class to compute whether a pose (dynamics) is within the interference
% free workspace (IFW)
%
% Author        : Paul Cheng
% Created       : 2020
% Description   :

classdef InterferenceFreeRayConditionCablePlaneObstacle < WorkspaceRayConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 9;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF;
    end
    
    properties (SetAccess = protected)
        % Set constants
        areDofsTranslation;         % Array for the q of the joint (true if translation and false if rotation)
        numDofs;                    % The number of dofs
        numCables;                  % The number of cables
        vertices;                   % The obstacle surface vertices
        connectivity;                % surface vertices connectivity
        is_compiled_mode;
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCablePlaneObstacle(model, min_ray_lengths, PolyObj)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.vertices = PolyObj{1};
            w.connectivity = PolyObj{2};
            w.is_compiled_mode = (model.modelMode == ModelModeType.COMPILED);
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            % Variable initialisation
            intervals = [];
            intervals_count = 1;
            %             tic
            free_variable_index = ws_ray.freeVariableIndex;
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            
            q_begin = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(1);ws_ray.fixedVariables(free_variable_index:end)];
            q_end = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(2);ws_ray.fixedVariables(free_variable_index:end)];
            
            all_intersection_poses = [q_begin,q_end];
            all_intersected_pts = [];
            % Cable_Surface_End_pts{1} -> base point,  Cable_Surface_End_pts{2}-> start point,  Cable_Surface_End_pts{3}-> end point
            [Cable_Surface_End_pts{1},Cable_Surface_End_pts{2}] = obj.GetSegmentData(model,q_begin);
            [~,Cable_Surface_End_pts{3}] = obj.GetSegmentData(model,q_end);
            for i = 1:size(obj.connectivity,1)
                vertices_i{i}  = obj.vertices(obj.connectivity(i,:),:);
            end
            [tri_u,tri_v,seg_t] = obj.RayTriangleEquation();
            
            if is_dof_translation
                for i = 1:obj.numCables
                    % three attachements from base frame
                    AttPts = [Cable_Surface_End_pts{1}(i,:);Cable_Surface_End_pts{2}(i,:);Cable_Surface_End_pts{3}(i,:)];
                    O_vector(i,:) = AttPts(2,:);
                    unit_vector(i,:) = (AttPts(3,:) - AttPts(2,:))/norm(AttPts(3,:) - AttPts(2,:));
                    ray_o(:,i) = Cable_Surface_End_pts{2}(i,:)';
                    ray_d(:,i) = unit_vector(i,:)';
                end
                
                [v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3] = obj.VarSubs(O_vector,unit_vector, vertices_i);
                
                % find the intersection of the ray to the triangle
                % surfaces
                u =tri_u(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                %                     u = u(:);
                v =tri_v(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                %                     v = v(:);
                t = seg_t(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                flag = double(u>= 0) + double(v>=0) + double(v+u <=1);
                %                         + double(t>= 0) + double(t <= 1)
                [intersected_tri_index,intersected_cable] =  find(flag == 3);
                
                if ~isempty(intersected_tri_index)
                    
                    for j = 1:size(intersected_tri_index,1)
                        tmp_val(:,j) = round(ray_o(:,intersected_cable(j)) + ray_d(:,intersected_cable(j)) .*t(intersected_tri_index(j),intersected_cable(j)),obj.ROUNDING_DIGIT)';
                        if  tmp_val(1,j) <= max(vertices_i{intersected_tri_index(j)}(:,1)) && tmp_val(1,j)>= min(vertices_i{intersected_tri_index(j)}(:,1)) &&...
                                tmp_val(2,j) <= max(vertices_i{intersected_tri_index(j)}(:,2)) && tmp_val(2,j)>= min(vertices_i{intersected_tri_index(j)}(:,2)) &&...
                                tmp_val(3,j) <= max(vertices_i{intersected_tri_index(j)}(:,3)) && tmp_val(3,j)>= min(vertices_i{intersected_tri_index(j)}(:,3))
                            all_intersection_poses = [all_intersection_poses,(q_end - q_begin)*t(intersected_tri_index(j)) + q_begin];
                            all_intersected_pts = [all_intersected_pts, tmp_val(:,j)];
                        end
                    end
                end
                
            else
                
                valid_range = [tan(q_begin(free_variable_index)/2), tan(q_end(free_variable_index)/2)];
                [Si,rot_axis] = GetSegmentEquation(obj,model,q_begin,q_end);
                
                u_coeff_quad = obj.GetUVCoeff4QuadOrient(rot_axis);
                q_zero = zeros(model.numDofs,1);
                if obj.is_compiled_mode
                    rOAi_all = model.cableModel.compiled_r_OAs_fn(q_begin,q_zero,q_zero,q_zero);
                else
                    rOAi_all = model.cableModel.r_OAs;
                end
                
                rOAi = rOAi_all(1:3,:);
                for i = 1:obj.numCables
                    rGAi(:,i) = model.cableModel.cables{1,i}.attachments{1,2}.r_GA;
                end
                parametric_cable_surf_uv = Si;
                [q_intersected,intersected_pts] = ParametericSurfaceIntersectionOrientation(obj,vertices_i,rOAi,rGAi,parametric_cable_surf_uv,...
                    q_begin,q_end,valid_range,u_coeff_quad,tri_u,tri_v,seg_t);
                
                all_intersection_poses = [all_intersection_poses, q_intersected];
                all_intersected_pts = [all_intersected_pts,intersected_pts];
                
            end
            
            %% verify the intersection interval
            all_intersection_poses = unique(round(all_intersection_poses',obj.ROUNDING_DIGIT),'rows')';
            
            previous_interval_exist = 0; %no interval before
            
            for i = 1:size(all_intersection_poses,2) - 1
                q_test = (all_intersection_poses(:,i)+all_intersection_poses(:,i+1))/2;
                q_zero = zeros(model.numDofs,1);
                if obj.is_compiled_mode
                    rOAi_all = model.cableModel.compiled_r_OAs_fn(q_test,q_zero,q_zero,q_zero);
                    length = model.cableModel.compiled_lengths_fn(q_test,q_zero,q_zero,q_zero);
                else
                    model.update(q_test,q_zero,q_zero,q_zero);
                    rOAi_all = model.cableModel.r_OAs;
                    length = model.cableLengths;
                end
                
                rOAi_i= rOAi_all(1:3,:);
                rOAi_j= rOAi_all(4:6,:);
                Si_unit_all =   (rOAi_j - rOAi_i)./length';
                
                [v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3] = obj.VarSubs(rOAi_i', Si_unit_all', vertices_i);
                u = tri_u(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                v = tri_v(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                %                     t = seg_t(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                flag = double(u>= 0) + double(v>=0) + double(v+u <=1) ;
                %                         + double(t>= 0) + double(t <= 1);
                [tri_ind,cable_ind] = find(flag == 3);
                is_intersected =  ~isempty(cable_ind);
                % if not empty, there is intersection, the the flag
                % is 1
                
                
                if ~is_intersected
                    if ~isempty(intervals) && intervals(end) == all_intersection_poses(free_variable_index,i)
                        intervals(end) = all_intersection_poses(free_variable_index,i+1);
                    else
                        intervals(intervals_count,:) = [all_intersection_poses(free_variable_index,i),all_intersection_poses(free_variable_index,i+1)];
                        intervals_count = intervals_count + 1;
                    end
                    
                end
                
            end
            
            %             if ~isempty(all_intersected_pts)
            %                 scatter3(all_intersected_pts(1,:),all_intersected_pts(2,:),all_intersected_pts(3,:),'r','filled');
            %             end
        end
    end
    methods (Access = private)
        %functin to get the cable segment data
        function [base_att_pt,EE_att_pt] = GetSegmentData(obj,model,q)
            q_zero = zeros(model.numDofs,1);
            if obj.is_compiled_mode
                rOAi = model.cableModel.compiled_r_OAs_fn(q,q_zero,q_zero,q_zero);
                base_att_pt =  rOAi(1:3,:)';
                EE_att_pt = rOAi(4:6,:)';
            else
                model.update(q,q_zero,q_zero,q_zero);
                rOAi = model.cableModel.r_OAs;
                base_att_pt =  rOAi(1:3,:)';
                EE_att_pt = rOAi(4:6,:)';
            end
        end
        
        %% function to get the cable segment equation
        function [Si,Rotational_axis_ind] = GetSegmentEquation(obj,model,q_begin,q_end)
            % codes here may need to be change according to the actually
            % situtation since it's difficult to find to which axis is
            % rotating from just the model
            Rotational_axis_ind =  find(~(q_begin == q_end));
            %             Rotational_axis_ind = [4,5,6];
            %             model.update(q_end,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
            %             S1 = model.cableModel.cables{1, 1}.segments{1,1}.segmentVector;
            %             model.update(q_begin,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
            %             S2 = model.cableModel.cables{1, 1}.segments{1,1}.segmentVector;
            %             Rotational_axis_ind(S2 == S1);
            % 4: rotate about x-axis 5: rotate about x-axis 6: rotate about x-axis
            if Rotational_axis_ind == 4
                Rot_Mat =@(u) [1 0 0;
                    0 (1-u^2)/(1+u^2) -2*u/(1+u^2);
                    0 2*u/(1+u^2) (1-u^2)/(1+u^2)];
            elseif Rotational_axis_ind == 5
                Rot_Mat =@(u) [(1-u^2)/(1+u^2) 0 2*u/(1+u^2);
                    0 1 0 ;
                    -2*u/(1+u^2) 0 (1-u^2)/(1+u^2)];
            elseif Rotational_axis_ind == 6
                Rot_Mat =@(u) [(1-u^2)/(1+u^2) -2*u/(1+u^2) 0;
                    2*u/(1+u^2) (1-u^2)/(1+u^2) 0;
                    0 0 1];
            else
                CASPR_log.Error('Cannot find rotational axis of this ray')
            end
            if obj.is_compiled_mode
                q_zero = zeros(model.numDofs,1);
                rOAi_all = model.cableModel.compiled_r_OAs_fn(q_begin,q_zero,q_zero,q_zero);
            else
                rOAi_all = model.cableModel.r_OAs;
            end
            for i = 1:model.numCables
                r_OA_i = rOAi_all(1:3,i);
                r_GA_i = model.cableModel.cables{1,i}.attachments{1,2}.r_GA;
                Si{i} =@(u) (q_begin(obj.areDofsTranslation) - r_OA_i) + Rot_Mat(u)*r_GA_i;
            end
        end
        
        %% function to calculate the intersected poses between quad-surface and cable segment bounded surface
        function [q_intersected,intersected_pts] = ParametericSurfaceIntersectionOrientation(obj,vertices,r_OA_i,r_GA_i,uv_equ,q_begin,q_end,u_range,u_coeff_u,tri_u,tri_v,seg_t)
            intersected_pts = [];q_intersected = [];
            %%
            q = q_begin(obj.areDofsTranslation);
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            
            a1 = r_OA_i(1,:);
            a2 = r_OA_i(2,:);
            a3 = r_OA_i(3,:);
            b1 = r_GA_i(1,:);
            b2 = r_GA_i(2,:);
            b3 = r_GA_i(3,:);
            
            
            tri_edge_pattern = [3,2;3,1;2,1];
            
            for i = 1:size(tri_edge_pattern,1)
                for j = 1:12
                    k11(j,i) = vertices{j}(tri_edge_pattern(i,1),1);
                    k12(j,i) = vertices{j}(tri_edge_pattern(i,1),2);
                    k13(j,i) = vertices{j}(tri_edge_pattern(i,1),3);
                    k21(j,i) = vertices{j}(tri_edge_pattern(i,2),1);
                    k22(j,i) = vertices{j}(tri_edge_pattern(i,2),2);
                    k23(j,i) = vertices{j}(tri_edge_pattern(i,2),3);
                end
            end
            k11 = k11(:);k12 = k12(:);k13 = k13(:);k21 = k21(:);k22 = k22(:);k23 = k23(:);
            tmp_u_value = [];
            u_coeff = [];
            
            u_coeff = round(u_coeff_u(a1,a2,a3,b1,b2,b3,k11,k12,k13,k21,k22,k23,q1,q2,q3),obj.ROUNDING_DIGIT);
            tmp_u_value_1 = @(a,b,c) (-b + sqrt(b.^2-4*a.*c))./(2*a);
            tmp_u_value_2 = @(a,b,c) (-b - sqrt(b.^2-4*a.*c))./(2*a);
            tmp_u_value = [tmp_u_value_1(u_coeff(:,1),u_coeff(:,2),u_coeff(:,3));tmp_u_value_2(u_coeff(:,1),u_coeff(:,2),u_coeff(:,3))];
            
            tmp_u_value = tmp_u_value(imag(tmp_u_value)==0);
            u_value = unique(tmp_u_value);
            u_value(u_value <u_range(1)) = [];u_value(u_value > u_range(2)) = [];
            
            %             dfd = uv_equ(u_value) + r_OA_i
            %             line([dfd(1);1],[dfd(2);0],[dfd(3);0])
            %             syms r u v t k11 k12 k13 k21 k22 k23 a1 a2 a3 b1 b2 b3 q1 q2 q3
            %             f1 = uv_equ(u)
            %             rayt = ([k11;k12;k13] -  [k21;k22;k23]);
            %             f2 = cross(f1,rayt)
            %             f3 = f2.'*[r_OA_i - [k11;k12;k13]]
            %             [f3n,f3d] = numden(f3)
            %             [f3c,f3deg]=coeffs(f3n,u)
            %             solve(f3c)
            %             f3 = f2v(2)
            %             [f3v,f3vd] = coeffs(f3,u);
            %             f3uans = -f3v(2) + sqrt(f3v(2)^2)

            if ~isempty(u_value)
                for i = 1:size(u_value,1)
                    %%% find the intersection point
                    for j = 1:size(uv_equ,2)
                        Si_unit(j,:) =  uv_equ{j}(u_value(i))';
                    end
                    [v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3] = obj.VarSubs(r_OA_i', Si_unit, vertices);
                    u = tri_u(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                    v = tri_v(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                    t = seg_t(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3);
                    flag = double(u>= 0) + double(v>=0) + double(v+u <=1) ;
                    [tri_ind,cable_ind] = find(flag == 3);
                    
                    ray_d = Si_unit';
                    ray_o = r_OA_i;
                    if ~isempty(tri_ind)
                        for j = 1:size(tri_ind,1)
                            tmp_val(:,j) = round(ray_o(:,cable_ind(j)) + ray_d(:,cable_ind(j)) .*t(tri_ind(j),cable_ind(j)),obj.ROUNDING_DIGIT)';
                            
                            if  tmp_val(1,j) <= max(vertices{tri_ind(j)}(:,1)) && tmp_val(1,j)>= min(vertices{tri_ind(j)}(:,1)) &&...
                                    tmp_val(2,j) <= max(vertices{tri_ind(j)}(:,2)) && tmp_val(2,j)>= min(vertices{tri_ind(j)}(:,2)) &&...
                                    tmp_val(3,j) <= max(vertices{tri_ind(j)}(:,3)) && tmp_val(3,j)>= min(vertices{tri_ind(j)}(:,3))
                                q_intersected = [q_intersected,(q_end - q_begin)*interp1(u_range,[0 1],u_value(i)) + q_begin];
                                intersected_pts = [intersected_pts, tmp_val(:,j)];
                            end
                        end
                    end
                end
            end

        end
        
        %% function to get the u coefficient in order to speed up the calculation
        function [u_coeff] = GetUVCoeff4QuadOrient(~,rot_axis)
            u_coeff = [];
            if rot_axis == 4 % x-axis
                u_coeff =@(a1,a2,a3,b1,b2,b3,k11,k12,k13,k21,k22,k23,q1,q2,q3)...
                    [ reshape(a1.*b3.*k12 - a1.*b2.*k13 - a2.*b1.*k13 - a2.*b3.*k11 + a3.*b1.*k12 + a3.*b2.*k11 + a1.*b2.*k23 - a1.*b3.*k22 + a2.*b1.*k23 + a2.*b3.*k21 - a3.*b1.*k22 - a3.*b2.*k21 + a1.*k12.*k23 - a1.*k13.*k22 - a2.*k11.*k23 + a2.*k13.*k21 + a3.*k11.*k22 - a3.*k12.*k21 - b1.*k12.*k23 + b1.*k13.*k22 - b2.*k11.*k23 + b2.*k13.*k21 + b3.*k11.*k22 - b3.*k12.*k21 - a1.*k12*q3 + a1.*k13*q2 + a2.*k11*q3 - a2.*k13*q1 - a3.*k11*q2 + a3.*k12*q1 + a1.*k22*q3 - a1.*k23*q2 - a2.*k21*q3 + a2.*k23*q1 + a3.*k21*q2 - a3.*k22*q1 - k11.*k22*q3 + k11.*k23*q2 + k12.*k21*q3 - k12.*k23*q1 - k13.*k21*q2 + k13.*k22*q1,[size(k12,1)*size(a3,2),1]),...
                    reshape(2*a2.*b2.*k11 - 2*a1.*b2.*k12 - 2*a1.*b3.*k13 + 2*a3.*b3.*k11 + 2*a1.*b2.*k22 - 2*a2.*b2.*k21 + 2*a1.*b3.*k23 - 2*a3.*b3.*k21 - 2.*b2.*k11.*k22 + 2.*b2.*k12.*k21 - 2.*b3.*k11.*k23 + 2.*b3.*k13.*k21,[size(k12,1)*size(a3,2),1]),...
                    reshape(a1.*b2.*k13 - a1.*b3.*k12 - a2.*b1.*k13 + a2.*b3.*k11 + a3.*b1.*k12 - a3.*b2.*k11 - a1.*b2.*k23 + a1.*b3.*k22 + a2.*b1.*k23 - a2.*b3.*k21 - a3.*b1.*k22 + a3.*b2.*k21 + a1.*k12.*k23 - a1.*k13.*k22 - a2.*k11.*k23 + a2.*k13.*k21 + a3.*k11.*k22 - a3.*k12.*k21 - b1.*k12.*k23 + b1.*k13.*k22 + b2.*k11.*k23 - b2.*k13.*k21 - b3.*k11.*k22 + b3.*k12.*k21 - a1.*k12*q3 + a1.*k13*q2 + a2.*k11*q3 - a2.*k13*q1 - a3.*k11*q2 + a3.*k12*q1 + a1.*k22*q3 - a1.*k23*q2 - a2.*k21*q3 + a2.*k23*q1 + a3.*k21*q2 - a3.*k22*q1 - k11.*k22*q3 + k11.*k23*q2 + k12.*k21*q3 - k12.*k23*q1 - k13.*k21*q2 + k13.*k22*q1,[size(k12,1)*size(a3,2),1])];
            elseif rot_axis == 5 %y-axis
                u_coeff =@(a1,a2,a3,b1,b2,b3,k11,k12,k13,k21,k22,k23,q1,q2,q3)...
                    [ reshape(a1.*b2.*k13 + a1.*b3.*k12 + a2.*b1.*k13 - a2.*b3.*k11 - a3.*b1.*k12 - a3.*b2.*k11 - a1.*b2.*k23 - a1.*b3.*k22 - a2.*b1.*k23 + a2.*b3.*k21 + a3.*b1.*k22 + a3.*b2.*k21 + a1.*k12.*k23 - a1.*k13.*k22 - a2.*k11.*k23 + a2.*k13.*k21 + a3.*k11.*k22 - a3.*k12.*k21 + b1.*k12.*k23 - b1.*k13.*k22 + b2.*k11.*k23 - b2.*k13.*k21 + b3.*k11.*k22 - b3.*k12.*k21 - a1.*k12*q3 + a1.*k13*q2 + a2.*k11*q3 - a2.*k13*q1 - a3.*k11*q2 + a3.*k12*q1 + a1.*k22*q3 - a1.*k23*q2 - a2.*k21*q3 + a2.*k23*q1 + a3.*k21*q2 - a3.*k22*q1 - k11.*k22*q3 + k11.*k23*q2 + k12.*k21*q3 - k12.*k23*q1 - k13.*k21*q2 + k13.*k22*q1,[size(k12,1)*size(a3,2),1]),...
                    reshape(2*a1.*b1.*k12 - 2*a2.*b1.*k11 - 2*a2.*b3.*k13 + 2*a3.*b3.*k12 - 2*a1.*b1.*k22 + 2*a2.*b1.*k21 + 2*a2.*b3.*k23 - 2*a3.*b3.*k22 + 2.*b1.*k11.*k22 - 2.*b1.*k12.*k21 - 2.*b3.*k12.*k23 + 2.*b3.*k13.*k22,[size(k12,1)*size(a3,2),1]),...
                    reshape(a1.*b2.*k13 - a1.*b3.*k12 - a2.*b1.*k13 + a2.*b3.*k11 + a3.*b1.*k12 - a3.*b2.*k11 - a1.*b2.*k23 + a1.*b3.*k22 + a2.*b1.*k23 - a2.*b3.*k21 - a3.*b1.*k22 + a3.*b2.*k21 + a1.*k12.*k23 - a1.*k13.*k22 - a2.*k11.*k23 + a2.*k13.*k21 + a3.*k11.*k22 - a3.*k12.*k21 - b1.*k12.*k23 + b1.*k13.*k22 + b2.*k11.*k23 - b2.*k13.*k21 - b3.*k11.*k22 + b3.*k12.*k21 - a1.*k12*q3 + a1.*k13*q2 + a2.*k11*q3 - a2.*k13*q1 - a3.*k11*q2 + a3.*k12*q1 + a1.*k22*q3 - a1.*k23*q2 - a2.*k21*q3 + a2.*k23*q1 + a3.*k21*q2 - a3.*k22*q1 - k11.*k22*q3 + k11.*k23*q2 + k12.*k21*q3 - k12.*k23*q1 - k13.*k21*q2 + k13.*k22*q1,[size(k12,1)*size(a3,2),1])];
            elseif rot_axis ==6
                u_coeff =@(a1,a2,a3,b1,b2,b3,k11,k12,k13,k21,k22,k23,q1,q2,q3)...
                    [ reshape(a2.*b1.*k13 - a1.*b3.*k12 - a1.*b2.*k13 + a2.*b3.*k11 - a3.*b1.*k12 + a3.*b2.*k11 + a1.*b2.*k23 + a1.*b3.*k22 - a2.*b1.*k23 - a2.*b3.*k21 + a3.*b1.*k22 - a3.*b2.*k21 + a1.*k12.*k23 - a1.*k13.*k22 - a2.*k11.*k23 + a2.*k13.*k21 + a3.*k11.*k22 - a3.*k12.*k21 + b1.*k12.*k23 - b1.*k13.*k22 - b2.*k11.*k23 + b2.*k13.*k21 - b3.*k11.*k22 + b3.*k12.*k21 - a1.*k12*q3 + a1.*k13*q2 + a2.*k11*q3 - a2.*k13*q1 - a3.*k11*q2 + a3.*k12*q1 + a1.*k22*q3 - a1.*k23*q2 - a2.*k21*q3 + a2.*k23*q1 + a3.*k21*q2 - a3.*k22*q1 - k11.*k22*q3 + k11.*k23*q2 + k12.*k21*q3 - k12.*k23*q1 - k13.*k21*q2 + k13.*k22*q1,[size(k12,1)*size(a3,2),1]),...
                    reshape(2*a1.*b1.*k13 - 2*a3.*b1.*k11 + 2*a2.*b2.*k13 - 2*a3.*b2.*k12 - 2*a1.*b1.*k23 + 2*a3.*b1.*k21 - 2*a2.*b2.*k23 + 2*a3.*b2.*k22 + 2.*b1.*k11.*k23 - 2.*b1.*k13.*k21 + 2.*b2.*k12.*k23 - 2.*b2.*k13.*k22,[size(k12,1)*size(a3,2),1])...
                    reshape(a1.*b2.*k13 - a1.*b3.*k12 - a2.*b1.*k13 + a2.*b3.*k11 + a3.*b1.*k12 - a3.*b2.*k11 - a1.*b2.*k23 + a1.*b3.*k22 + a2.*b1.*k23 - a2.*b3.*k21 - a3.*b1.*k22 + a3.*b2.*k21 + a1.*k12.*k23 - a1.*k13.*k22 - a2.*k11.*k23 + a2.*k13.*k21 + a3.*k11.*k22 - a3.*k12.*k21 - b1.*k12.*k23 + b1.*k13.*k22 + b2.*k11.*k23 - b2.*k13.*k21 - b3.*k11.*k22 + b3.*k12.*k21 - a1.*k12*q3 + a1.*k13*q2 + a2.*k11*q3 - a2.*k13*q1 - a3.*k11*q2 + a3.*k12*q1 + a1.*k22*q3 - a1.*k23*q2 - a2.*k21*q3 + a2.*k23*q1 + a3.*k21*q2 - a3.*k22*q1 - k11.*k22*q3 + k11.*k23*q2 + k12.*k21*q3 - k12.*k23*q1 - k13.*k21*q2 + k13.*k22*q1,[size(k12,1)*size(a3,2),1])];
            end
            
        end
        %% function to calculate the intersection of a segment and triangle
        function [flag, u, v, t] = SegmentTriangleIntersection (~,o, d, vertrics)
            % Ray/triangle intersection using the algorithm proposed by Möller and Trumbore (1997).
            %
            % Input:
            % o : origin.
            % d : direction.
            % p0, p1, p2: vertices of the triangle.
            % Output:
            % flag: (0) Reject, (1) Intersect.
            % u,v: barycentric coordinates.
            % t: distance from the ray origin.
            % Author:
            % Originally written by Jesus Mena, edited by David Berman (dberm22@gmail.com)
            % %             syms v1 v2 v3 v4 v5 v6 v7 v8 v9 o1 o2 o3 d1 d2 d3
            %             vertrics = [v1 v2 v3;v4 v5 v6;v7 v8 v9]
            %             o = [o1 o2 o3];
            %             d = [d1 d2 d3];
            
            o = reshape(o,[1,3]);
            d = reshape(d,[1,3]);
            epsilon = 0.00001;
            p0 = vertrics(1,:);
            p1 = vertrics(2,:);
            p2 = vertrics(3,:);
            e1 = p1-p0;
            e2 = p2-p0;
            q = [d(2)*e2(3)-d(3)*e2(2), d(3)*e2(1)-d(1)*e2(3), d(1)*e2(2)-d(2)*e2(1)]; %cross product
            a = e1(1)*q(1)+e1(2)*q(2)+e1(3)*q(3); % determinant of the matrix M
            
            if (a>-epsilon && a<epsilon)
                % the vector is parallel to the plane (the intersection is at infinity)
                flag=0;
                u=0;
                v=0;
                t=0;
                return;
            end
            
            f = 1/a;
            s = o-p0;
            u = f*(s(1)*q(1)+s(2)*q(2)+s(3)*q(3));
            
            if (u<0.0)
                % the intersection is outside of the triangle
                flag=0;
                u=0;
                v=0;
                t=0;
                return;
            end
            
            r = [s(2)*e1(3)-s(3)*e1(2), s(3)*e1(1)-s(1)*e1(3), s(1)*e1(2)-s(2)*e1(1)];
            v = f*(d(1)*r(1)+d(2)*r(2)+d(3)*r(3));
            
            if (v<0.0 || u+v>1.0)
                % the intersection is outside of the triangle
                flag=0;
                u=0;
                v=0;
                t=0;
                return;
            end
            
            t = f*(e2(1)*r(1)+e2(2)*r(2)+e2(3)*r(3)); % verified!
            flag = 1;
            return;
        end
        
        function [u,v,t] = RayTriangleEquation(~)
            u = @(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3)...
                -((d2'.*(v1 - v7) - d1'.*(v2 - v8)).*(o3' - v3) - (d3'.*(v1 - v7) - d1'.*(v3 - v9)).*(o2' - v2) + (d3'.*(v2 - v8) - d2'.*(v3 - v9)).*(o1' - v1))./((d2'.*(v1 - v7) - d1'.*(v2 - v8)).*(v3 - v6) - (d3'.*(v1 - v7) - d1'.*(v3 - v9)).*(v2 - v5) + (d3'.*(v2 - v8) - d2'.*(v3 - v9)).*(v1 - v4));
            v = @(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3)...
                (d3'.*((o1' - v1).*(v2 - v5) - (o2' - v2).*(v1 - v4)) - d2'.*((o1' - v1).*(v3 - v6) - (o3' - v3).*(v1 - v4)) + d1'.*((o2' - v2).*(v3 - v6) - (o3' - v3).*(v2 - v5)))./((d2'.*(v1 - v7) - d1'.*(v2 - v8)).*(v3 - v6) - (d3'.*(v1 - v7) - d1'.*(v3 - v9)).*(v2 - v5) + (d3'.*(v2 - v8) - d2'.*(v3 - v9)).*(v1 - v4));
            t = @(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3)...
                -((v3 - v9).*((o1' - v1).*(v2 - v5) - (o2' - v2).*(v1 - v4)) - (v2 - v8).*((o1' - v1).*(v3 - v6) - (o3' - v3).*(v1 - v4)) + (v1 - v7).*((o2' - v2).*(v3 - v6) - (o3' - v3).*(v2 - v5)))./((d2'.*(v1 - v7) - d1'.*(v2 - v8)).*(v3 - v6) - (d3'.*(v1 - v7) - d1'.*(v3 - v9)).*(v2 - v5) + (d3'.*(v2 - v8) - d2'.*(v3 - v9)).*(v1 - v4));
            
            %             u = @(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3)...
            %                 -((d2.*(v1 - v7) - d1.*(v2 - v8)).*(o3 - v3) - (d3.*(v1 - v7) - d1.*(v3 - v9)).*(o2 - v2) + (d3.*(v2 - v8) - d2.*(v3 - v9)).*(o1 - v1))./((d2.*(v1 - v7) - d1.*(v2 - v8)).*(v3 - v6) - (d3.*(v1 - v7) - d1.*(v3 - v9)).*(v2 - v5) + (d3.*(v2 - v8) - d2.*(v3 - v9)).*(v1 - v4));
            %             v = @(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3)...
            %                 (d3.*((o1 - v1).*(v2 - v5) - (o2 - v2).*(v1 - v4)) - d2.*((o1 - v1).*(v3 - v6) - (o3 - v3).*(v1 - v4)) + d1.*((o2 - v2).*(v3 - v6) - (o3 - v3).*(v2 - v5)))./((d2.*(v1 - v7) - d1.*(v2 - v8)).*(v3 - v6) - (d3.*(v1 - v7) - d1.*(v3 - v9)).*(v2 - v5) + (d3.*(v2 - v8) - d2.*(v3 - v9)).*(v1 - v4));
            %             t = @(v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3)...
            %                 -((v3 - v9).*((o1 - v1).*(v2 - v5) - (o2 - v2).*(v1 - v4)) - (v2 - v8).*((o1 - v1).*(v3 - v6) - (o3 - v3).*(v1 - v4)) + (v1 - v7).*((o2 - v2).*(v3 - v6) - (o3 - v3).*(v2 - v5)))./((d2.*(v1 - v7) - d1.*(v2 - v8)).*(v3 - v6) - (d3.*(v1 - v7) - d1.*(v3 - v9)).*(v2 - v5) + (d3.*(v2 - v8) - d2.*(v3 - v9)).*(v1 - v4));
            
        end
        
        function [v1,v2,v3,v4,v5,v6,v7,v8,v9,o1,o2,o3,d1,d2,d3] = VarSubs(~,var1,var2,var3)
            %input var1,var2 -> n x 3, var3 -> 1 x m
            for i = 1:size(var1,1)
                o1(i,:) = var1(i,1);o2(i,:) = var1(i,2);o3(i,:) = var1(i,3);
                d1(i,:) = var2(i,1);d2(i,:) = var2(i,2);d3(i,:) = var2(i,3);
            end
            %             o1 = var1(1);o2 = var1(2);o3 = var1(3);
            %             d1 = var2(1);d2 = var2(2);d3 = var2(3);
            for i = 1:size(var3,2)
                v1(i,:) = var3{i}(1,1);
                v2(i,:) = var3{i}(1,2);
                v3(i,:) = var3{i}(1,3);
                v4(i,:) = var3{i}(2,1);
                v5(i,:) = var3{i}(2,2);
                v6(i,:) = var3{i}(2,3);
                v7(i,:) = var3{i}(3,1);
                v8(i,:) = var3{i}(3,2);
                v9(i,:) = var3{i}(3,3);
            end
        end
        
    end
end


