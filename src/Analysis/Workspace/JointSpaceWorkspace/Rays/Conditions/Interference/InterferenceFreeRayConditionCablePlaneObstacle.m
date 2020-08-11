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
        connectivity                % surface vertices connectivity
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
            
            
            for triangle_ind = 1:size(obj.connectivity,1)
                vertices_i  = obj.vertices(obj.connectivity(triangle_ind,:),:);
                
                if is_dof_translation
                    
                    for i = 1:obj.numCables
                        % three attachements from base frame
                        AttPts = [Cable_Surface_End_pts{1}(i,:);Cable_Surface_End_pts{2}(i,:);Cable_Surface_End_pts{3}(i,:)];
                        % find the intersection of the ray to the triangle
                        % surfaces
                        
                        [flag, ~,~, t] = obj.SegmentTriangleIntersection(AttPts(2,:), (AttPts(3,:) - AttPts(2,:))/norm(AttPts(3,:) - AttPts(2,:)), vertices_i);
                        
                        if flag == 1
                            tmp_val = round(AttPts(2,:) + (AttPts(3,:) - AttPts(2,:))/norm(AttPts(3,:) - AttPts(2,:))*t,obj.ROUNDING_DIGIT)';
                            
                            if tmp_val(1) <= max(vertices_i(:,1)) && tmp_val(1)>= min(vertices_i(:,1)) &&...
                                    tmp_val(2) <= max(vertices_i(:,2)) && tmp_val(2)>= min(vertices_i(:,2)) &&...
                                    tmp_val(3) <= max(vertices_i(:,3)) && tmp_val(3)>= min(vertices_i(:,3))
                                all_intersection_poses = [all_intersection_poses,(q_end - q_begin)*t + q_begin];
                                all_intersected_pts = [all_intersected_pts, tmp_val];
                            end
                        end
                        
                    end
                    
                else
                    
                    valid_range = [tan(q_begin(free_variable_index)/2), tan(q_end(free_variable_index)/2)];
                    [Si,rot_axis] = GetSegmentEquation(obj,model,q_begin,q_end);
                    
                    u_coeff_quad = obj.GetUVCoeff4QuadOrient(rot_axis);
                    
                    for i = 1:obj.numCables
                        
                        rOAi = model.cableModel.cables{1,i}.attachments{1,1}.r_OA;
                        rGAi = model.cableModel.cables{1,i}.attachments{1,2}.r_GA;
                        AttPts = [Cable_Surface_End_pts{1}(i,:);Cable_Surface_End_pts{2}(i,:);Cable_Surface_End_pts{3}(i,:)];
                        Si_u =@(u) Si{i}(u);
                        
                        parametric_cable_surf_uv = Si_u;
                        [q_intersected,intersected_pts] = ParametericSurfaceIntersectionOrientation(obj,vertices_i,rOAi,rGAi,parametric_cable_surf_uv,...
                            q_begin,q_end,valid_range,u_coeff_quad);
                        
                        all_intersection_poses = [all_intersection_poses, q_intersected];
                        all_intersected_pts = [all_intersected_pts,intersected_pts];
                        
                    end
                end
                
                %% verify the intersection interval
                all_intersection_poses = unique(round(all_intersection_poses',obj.ROUNDING_DIGIT),'rows')';
            end
            pre_has_intersected = 1;
            for i = 1:size(all_intersection_poses,2) - 1
                q_test = (all_intersection_poses(:,i)+all_intersection_poses(:,i+1))/2;
                model.update(q_test,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
                for j = 1:obj.numCables
                    Si_unit = model.cableModel.cables{1,j}.segments{1,1}.segmentVector/norm(model.cableModel.cables{1,j}.segments{1,1}.segmentVector);
                    rOAi  = model.cableModel.cables{1,j}.attachments{1, 1}.r_OA;
                    for triangle_ind = 1:size(obj.connectivity,1)
                        vertices_i  = obj.vertices(obj.connectivity(triangle_ind,:),:);
                        [flag(j,triangle_ind),~,~,~] = obj.SegmentTriangleIntersection (rOAi, model.cableModel.cables{1,j}.segments{1,1}.segmentVector, vertices_i);
                        if flag(j,triangle_ind) == 1
                            break
                        end
                    end
                end
                if ~any(flag(:))
                    if pre_has_intersected == 1
                        intervals(intervals_count,:) = [all_intersection_poses(free_variable_index,i),all_intersection_poses(free_variable_index,i+1)];
                        intervals_count = intervals_count + 1;
                    else
                        intervals(intervals_count-1 ,end) = all_intersection_poses(free_variable_index,i+1);
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
        function [base_att_pt,EE_att_pt] = GetSegmentData(~,model,q)
            
            model.update(q,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1))
            for i = 1:model.numCables
                base_att_pt(i,:) = model.cableModel.cables{1,i}.attachments{1,1}.r_OA';
                EE_att_pt(i,:) = model.cableModel.cables{1,i}.attachments{1,2}.r_OA';
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
            %                         syms a1 a2 a3 b1 b2 b3 q1 q2 q3 u
            for i = 1:model.numCables
                r_OA_i = model.cableModel.cables{1,i}.attachments{1,1}.r_OA;
                r_GA_i = model.cableModel.cables{1,i}.attachments{1,2}.r_GA;
                %                                                 r_OA_i = [a1;a2;a3];
                %                                                 r_GA_i = [b1;b2;b3];
                Si{i} =@(u) (q_begin(obj.areDofsTranslation) - r_OA_i) + Rot_Mat(u)*r_GA_i;
                %                                                 Si{i} =@(u) ([q1;q2;q3] - r_OA_i) + Rot_Mat(u)*r_GA_i;
            end
        end
        
        %% function to calculate the intersected poses between quad-surface and cable segment bounded surface
        function [q_intersected,intersected_pts] = ParametericSurfaceIntersectionOrientation(obj,vertices,r_OA_i,r_GA_i,uv_equ,q_begin,q_end,u_range,u_coeff_u)
            intersected_pts = [];q_intersected = [];
            %%
            q = q_begin(obj.areDofsTranslation);
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            
            a1 = r_OA_i(1);
            a2 = r_OA_i(2);
            a3 = r_OA_i(3);
            b1 = r_GA_i(1);
            b2 = r_GA_i(2);
            b3 = r_GA_i(3);
            
            tri_edge_pattern = [3,2;3,1;2,1];
            tmp_u_value = [];
            
            for i = 1:size(tri_edge_pattern,1)
                k11 = vertices(tri_edge_pattern(i,1),1);
                k12 = vertices(tri_edge_pattern(i,1),2);
                k13 = vertices(tri_edge_pattern(i,1),3);
                k21 = vertices(tri_edge_pattern(i,2),1);
                k22 = vertices(tri_edge_pattern(i,2),2);
                k23 = vertices(tri_edge_pattern(i,2),3);
                u_coeff = round(u_coeff_u(a1,a2,a3,b1,b2,b3,k11,k12,k13,k21,k22,k23,q1,q2,q3),obj.ROUNDING_DIGIT);
                tmp_u_value = [tmp_u_value;roots(u_coeff)];
            end
            
            u_value = unique(tmp_u_value);
            u_value = u_value(imag(u_value)==0);
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
                    [flag, ~,~, t] = obj.SegmentTriangleIntersection (r_OA_i, uv_equ(u_value(i)), vertices);
                    
                    if flag == 1
                        tmp_val = round(r_OA_i + t*uv_equ(u_value(i)),obj.ROUNDING_DIGIT);
                        %                         g9g = scatter3(vertices(:,1),vertices(:,2),vertices(:,3),'filled')
                        %                         g8g = scatter3(tmp_val(1),tmp_val(2),tmp_val(3),'filled')
                        %                         delete(g8g)
                        %                         delete(g9g)
                        if tmp_val(1) <= max(vertices(:,1)) && tmp_val(1)>= min(vertices(:,1)) &&...
                                tmp_val(2) <= max(vertices(:,2)) && tmp_val(2)>= min(vertices(:,2)) &&...
                                tmp_val(3) <= max(vertices(:,3)) && tmp_val(3)>= min(vertices(:,3))
                            q_intersected = [q_intersected,(q_end - q_begin)*interp1(u_range,[0 1],u_value(i)) + q_begin];
                            intersected_pts = [intersected_pts, tmp_val];
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
                    [ a1*b3*k12 - a1*b2*k13 - a2*b1*k13 - a2*b3*k11 + a3*b1*k12 + a3*b2*k11 + a1*b2*k23 - a1*b3*k22 + a2*b1*k23 + a2*b3*k21 - a3*b1*k22 - a3*b2*k21 + a1*k12*k23 - a1*k13*k22 - a2*k11*k23 + a2*k13*k21 + a3*k11*k22 - a3*k12*k21 - b1*k12*k23 + b1*k13*k22 - b2*k11*k23 + b2*k13*k21 + b3*k11*k22 - b3*k12*k21 - a1*k12*q3 + a1*k13*q2 + a2*k11*q3 - a2*k13*q1 - a3*k11*q2 + a3*k12*q1 + a1*k22*q3 - a1*k23*q2 - a2*k21*q3 + a2*k23*q1 + a3*k21*q2 - a3*k22*q1 - k11*k22*q3 + k11*k23*q2 + k12*k21*q3 - k12*k23*q1 - k13*k21*q2 + k13*k22*q1, 2*a2*b2*k11 - 2*a1*b2*k12 - 2*a1*b3*k13 + 2*a3*b3*k11 + 2*a1*b2*k22 - 2*a2*b2*k21 + 2*a1*b3*k23 - 2*a3*b3*k21 - 2*b2*k11*k22 + 2*b2*k12*k21 - 2*b3*k11*k23 + 2*b3*k13*k21, a1*b2*k13 - a1*b3*k12 - a2*b1*k13 + a2*b3*k11 + a3*b1*k12 - a3*b2*k11 - a1*b2*k23 + a1*b3*k22 + a2*b1*k23 - a2*b3*k21 - a3*b1*k22 + a3*b2*k21 + a1*k12*k23 - a1*k13*k22 - a2*k11*k23 + a2*k13*k21 + a3*k11*k22 - a3*k12*k21 - b1*k12*k23 + b1*k13*k22 + b2*k11*k23 - b2*k13*k21 - b3*k11*k22 + b3*k12*k21 - a1*k12*q3 + a1*k13*q2 + a2*k11*q3 - a2*k13*q1 - a3*k11*q2 + a3*k12*q1 + a1*k22*q3 - a1*k23*q2 - a2*k21*q3 + a2*k23*q1 + a3*k21*q2 - a3*k22*q1 - k11*k22*q3 + k11*k23*q2 + k12*k21*q3 - k12*k23*q1 - k13*k21*q2 + k13*k22*q1];
            elseif rot_axis == 5 %y-axis
                u_coeff =@(a1,a2,a3,b1,b2,b3,k11,k12,k13,k21,k22,k23,q1,q2,q3)...
                    [ a1*b2*k13 + a1*b3*k12 + a2*b1*k13 - a2*b3*k11 - a3*b1*k12 - a3*b2*k11 - a1*b2*k23 - a1*b3*k22 - a2*b1*k23 + a2*b3*k21 + a3*b1*k22 + a3*b2*k21 + a1*k12*k23 - a1*k13*k22 - a2*k11*k23 + a2*k13*k21 + a3*k11*k22 - a3*k12*k21 + b1*k12*k23 - b1*k13*k22 + b2*k11*k23 - b2*k13*k21 + b3*k11*k22 - b3*k12*k21 - a1*k12*q3 + a1*k13*q2 + a2*k11*q3 - a2*k13*q1 - a3*k11*q2 + a3*k12*q1 + a1*k22*q3 - a1*k23*q2 - a2*k21*q3 + a2*k23*q1 + a3*k21*q2 - a3*k22*q1 - k11*k22*q3 + k11*k23*q2 + k12*k21*q3 - k12*k23*q1 - k13*k21*q2 + k13*k22*q1, 2*a1*b1*k12 - 2*a2*b1*k11 - 2*a2*b3*k13 + 2*a3*b3*k12 - 2*a1*b1*k22 + 2*a2*b1*k21 + 2*a2*b3*k23 - 2*a3*b3*k22 + 2*b1*k11*k22 - 2*b1*k12*k21 - 2*b3*k12*k23 + 2*b3*k13*k22, a1*b2*k13 - a1*b3*k12 - a2*b1*k13 + a2*b3*k11 + a3*b1*k12 - a3*b2*k11 - a1*b2*k23 + a1*b3*k22 + a2*b1*k23 - a2*b3*k21 - a3*b1*k22 + a3*b2*k21 + a1*k12*k23 - a1*k13*k22 - a2*k11*k23 + a2*k13*k21 + a3*k11*k22 - a3*k12*k21 - b1*k12*k23 + b1*k13*k22 + b2*k11*k23 - b2*k13*k21 - b3*k11*k22 + b3*k12*k21 - a1*k12*q3 + a1*k13*q2 + a2*k11*q3 - a2*k13*q1 - a3*k11*q2 + a3*k12*q1 + a1*k22*q3 - a1*k23*q2 - a2*k21*q3 + a2*k23*q1 + a3*k21*q2 - a3*k22*q1 - k11*k22*q3 + k11*k23*q2 + k12*k21*q3 - k12*k23*q1 - k13*k21*q2 + k13*k22*q1];
            elseif rot_axis ==6
                u_coeff =@(a1,a2,a3,b1,b2,b3,k11,k12,k13,k21,k22,k23,q1,q2,q3)...
                    [ a2*b1*k13 - a1*b3*k12 - a1*b2*k13 + a2*b3*k11 - a3*b1*k12 + a3*b2*k11 + a1*b2*k23 + a1*b3*k22 - a2*b1*k23 - a2*b3*k21 + a3*b1*k22 - a3*b2*k21 + a1*k12*k23 - a1*k13*k22 - a2*k11*k23 + a2*k13*k21 + a3*k11*k22 - a3*k12*k21 + b1*k12*k23 - b1*k13*k22 - b2*k11*k23 + b2*k13*k21 - b3*k11*k22 + b3*k12*k21 - a1*k12*q3 + a1*k13*q2 + a2*k11*q3 - a2*k13*q1 - a3*k11*q2 + a3*k12*q1 + a1*k22*q3 - a1*k23*q2 - a2*k21*q3 + a2*k23*q1 + a3*k21*q2 - a3*k22*q1 - k11*k22*q3 + k11*k23*q2 + k12*k21*q3 - k12*k23*q1 - k13*k21*q2 + k13*k22*q1, 2*a1*b1*k13 - 2*a3*b1*k11 + 2*a2*b2*k13 - 2*a3*b2*k12 - 2*a1*b1*k23 + 2*a3*b1*k21 - 2*a2*b2*k23 + 2*a3*b2*k22 + 2*b1*k11*k23 - 2*b1*k13*k21 + 2*b2*k12*k23 - 2*b2*k13*k22, a1*b2*k13 - a1*b3*k12 - a2*b1*k13 + a2*b3*k11 + a3*b1*k12 - a3*b2*k11 - a1*b2*k23 + a1*b3*k22 + a2*b1*k23 - a2*b3*k21 - a3*b1*k22 + a3*b2*k21 + a1*k12*k23 - a1*k13*k22 - a2*k11*k23 + a2*k13*k21 + a3*k11*k22 - a3*k12*k21 - b1*k12*k23 + b1*k13*k22 + b2*k11*k23 - b2*k13*k21 - b3*k11*k22 + b3*k12*k21 - a1*k12*q3 + a1*k13*q2 + a2*k11*q3 - a2*k13*q1 - a3*k11*q2 + a3*k12*q1 + a1*k22*q3 - a1*k23*q2 - a2*k21*q3 + a2*k23*q1 + a3*k21*q2 - a3*k22*q1 - k11*k22*q3 + k11*k23*q2 + k12*k21*q3 - k12*k23*q1 - k13*k21*q2 + k13*k22*q1];
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
        
        
        
    end
end


