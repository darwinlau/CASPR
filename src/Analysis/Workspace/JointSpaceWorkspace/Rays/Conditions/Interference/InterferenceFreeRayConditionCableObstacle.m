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
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            % Variable initialisation
            
            intervals = [];u_value = [];
            intervals_count = 1;
            free_variable_index = ws_ray.freeVariableIndex;
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            
            q_begin = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(1);ws_ray.fixedVariables(free_variable_index:end)];
            q_end = [ws_ray.fixedVariables(1:free_variable_index-1);ws_ray.freeVariableRange(2);ws_ray.fixedVariables(free_variable_index:end)];
            
            [obj.G_coeffs,obj.G_xyz] = obj.GetGxyz(model,q_begin,q_end,free_variable_index,is_dof_translation);
            [obj.OB_u,~]  = GetOB_u(obj,model,q_begin,q_end,free_variable_index,is_dof_translation);
            
            
            [OA_i_begin,OB_i_begin] = obj.GetSegmentData(model,q_begin);
            [OA_i_end,OB_i_end] = obj.GetSegmentData(model,q_end);
            
            for i = 1:size(OA_i_begin,1)
                [OA_i_hat(i,:),~] = obj.LineIntersection3D([OB_i_begin(i,:);OB_i_end(i,:)],[OA_i_begin(i,:);OA_i_end(i,:)]);
            end
            
            %%
%             tic
            u_seg_surf = [];
%             [q_begin,q_end]
            if is_dof_translation
                obj.u_range = [0 1];
                for surf_ind = 1:obj.obstacle.surfaceNum
                    if  obj.obstacle.surfaceDeg(surf_ind) ~= 1
                        [~,u_O,val_ind_1] = obj.Cable_Obstacle_Surface_Intersection(model,q_begin,q_end,surf_ind,free_variable_index,OA_i_hat);
                    else
                        u_O = [];val_ind_1 = [];
                    end                    
                    [~,u_L,val_ind_2] = obj.LineSegment_Surface_Intersection(OB_i_begin,OB_i_end,surf_ind,0);
                    
                    u_value = [u_value;u_O;u_L];
                    u_seg_surf = [u_seg_surf;val_ind_1;val_ind_2];
                end
                for bound_ind = 1:obj.obstacle.boundaryNum
                [~,u_G,v_val,segment_ind] = obj.SegmentIntersection_Gxyz(bound_ind,OA_i_hat,is_dof_translation);
                u_value = [u_value;u_G];
                u_seg_surf = [u_seg_surf;segment_ind];
                end
                [u_value,val_seg_surf] = obj.SortAnswerWithIndex(u_seg_surf,obj.u_range);
%                 toc
                q_intersected = (q_end - q_begin)*u_value' + q_begin;
                
            else
                obj.u_range = tan([q_begin(free_variable_index),q_end(free_variable_index)]/2);
                %% find surface surface/edge intersection
                 for surf_ind = 1:obj.obstacle.surfaceNum
                    if  obj.obstacle.surfaceDeg(surf_ind) ~= 1
                        [~,u_O,val_ind_1] = obj.Cable_Obstacle_Surface_Intersection(model,q_begin,q_end,surf_ind,free_variable_index,OA_i_hat);
                    else
                        u_O = [];val_ind_1 = [];
                    end                    
                     [~,u_L,val_ind_2] = obj.CurveSegment_Surface_Intersection(model,q_begin,q_end,surf_ind,free_variable_index);
              
                    u_value = [u_value;u_O;u_L];
                    u_seg_surf = [u_seg_surf;val_ind_1;val_ind_2];
                 end
                
                for bound_ind = 1:obj.obstacle.boundaryNum
                [~,u_G,v_val,segment_ind] = obj.SegmentIntersection_Gxyz(bound_ind,OA_i_hat,is_dof_translation);
                u_value = [u_value;u_G];
                u_seg_surf = [u_seg_surf;segment_ind];
                end
                
                [u_value,val_seg_surf] = obj.SortAnswerWithIndex(u_seg_surf,obj.u_range);
                
%                 
%                 for surf_ind = 1:obj.obstacle.surfaceNum
%                     if  obj.obstacle.surfaceDeg(surf_ind) ~= 1
%                         [~,u_O,u_seg_surf] = obj.Cable_Obstacle_Surface_Intersection(model,q_begin,q_end,surf_ind,free_variable_index,OA_i_hat);
%                     else
%                         u_O = [];u_seg_surf = [];
%                     end
%                     [~,u_L,segment_ind_curv] = obj.CurveSegment_Surface_Intersection(model,q_begin,q_end,surf_ind,free_variable_index);
%                 end
%                 u_value = [u_value;u_O;u_L];
%                 
%                 %                 toc
%                 %                 tic
%                                 [~,u_G,v_val,segment_ind] = obj.SegmentIntersection_Gxyz(OA_i_hat,is_dof_translation);
%                 %                 toc
%                 u_value = [u_value;u_G];
                q_intersected = repmat(q_begin,1,size(u_value,1));
                q_intersected(free_variable_index,:) = 2*atan(u_value);
            end
            %             toc
            %             tic
            q_intersected = [q_intersected,q_begin,q_end];
            %% verify the intersection interval
            q_intersected = unique(round(q_intersected',obj.ROUNDING_DIGIT),'rows')';
%            tic
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
            %             toc;
            
        end
    end
    methods (Access = private)
        %functin to get the cable segment data
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
        
        %function to find intersection between obstacle and cable swept surface
        function [flag,u_val,segment_ind] = Cable_Obstacle_Surface_Intersection(obj,model,q_begin,surf_ind,q_end,free_variable_index,OA_i_hat)
            u_coeff = [];v_coeff = [];
            
%             tic;
            u_roots_1 = [];u_coeff_c = [];
            for i = 1:obj.segment_num
                
                if obj.obstacle.surfaceDeg(surf_ind) == 4
                    a1 = H4_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b1 = H3_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c1 = H2_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    d1 = H1_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    e1 = H0_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff_c = [u_coeff_c;u_coeffs_4_deg(a1,b1,c1,d1,e1)];
                elseif obj.obstacle.surfaceDeg(surf_ind) == 3
                    a1 = H3_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b1 = H2_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c1 = H1_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    d1 = H0_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff_c = [u_coeff_c;u_coeffs_3_deg(a1,b1,c1,d1)];
                elseif obj.obstacle.surfaceDeg(surf_ind) == 2
                    a1 = H2_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    b1 = H1_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    c1 = H0_O(obj.OB_u{i},OA_i_hat(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                    u_coeff_c = [u_coeff_c;u_coeffs_2_deg(a1,b1,c1)];
                end
                
                
            end
%             toc;
            
            %             tic
            %             if ~obj.areDofsTranslation(free_variable_index) %orientation
            %                 sample_size =  obj.obstacle.surfaceDeg(surf_ind)*2 + 1;
            %                 q_sample = linspace(1.05*q_begin(free_variable_index),1.05*q_end(free_variable_index),sample_size);
            %                 u_sample = tan(q_sample/2);
            %                 for i = 0: obj.obstacle.surfaceDeg(surf_ind)
            %                     H_i_u_denominator(i+1,:) = (1+u_sample.^2).^i;
            %                 end
            %                 H_i_u_denominator = flipud(H_i_u_denominator);
            %                 H_i_deg =  obj.obstacle.surfaceDeg(surf_ind)*2:-2:0;
            %             else %translation
            %                 sample_size =  obj.obstacle.surfaceDeg(surf_ind) + 1;
            %                 q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
            %                 u_sample = linspace(0,1,sample_size);
            %                 H_i_deg =  obj.obstacle.surfaceDeg(surf_ind):-1:0;
            %                 for i = 0: obj.obstacle.surfaceDeg(surf_ind)
            %                     H_i_u_denominator(i+1,:) = ones(size(u_sample));
            %                 end
            %             end
            %             v = linspace(0,1,sample_size);
            %
            %             q_update = q_begin;
            %             for i = 1:size(q_sample,2)
            %                 q_update(free_variable_index) = q_sample(i);
            %                 [~,OB_i_u]  = obj.GetSegmentData(model,q_update);
            %                 for j = 1:size(v,2)
            %                     G_uv_v{j} = ((OB_i_u - OA_i_hat)*v(j) + OA_i_hat);
            %                 end
            %                 for j = 1:obj.segment_num
            %                     for k = 1:size(v,2)
            %                         v_sample(k) = obj.obstacle.surfaceEqu{surf_ind}(G_uv_v{k}(j,1),G_uv_v{k}(j,2),G_uv_v{k}(j,3));
            %
            %                     end
            %                     v_coeff{i}(:,j) = GeneralMathOperations.PolynomialFit(v', v_sample',  obj.obstacle.surfaceDeg(surf_ind));
            %
            %                 end
            %             end
            %             for j = 1:obj.segment_num
            %                 for i = 1:size(v_coeff,2)
            %                     H_i_u_sample(:,i) = v_coeff{i}(:,j) .* H_i_u_denominator(:,i);
            %                 end
            %                 for i = 1:size(H_i_u_sample,1)
            %                     H_i_coeff{i,:} = GeneralMathOperations.PolynomialFit(u_sample', H_i_u_sample(i,:)', H_i_deg(i))';
            %                 end
            %
            %                 if  obj.obstacle.surfaceDeg(surf_ind) == 2
            %                     a = H_i_coeff{1}; b = H_i_coeff{2}; c = H_i_coeff{3};
            %                     u_coeff(j,:) = obj.SumCoeff({conv(b,b);-4*a*c(end)});
            %                 elseif  obj.obstacle.surfaceDeg(surf_ind) == 3
            %                     a = H_i_coeff{1}; b = H_i_coeff{2}; c = H_i_coeff{3}; d = H_i_coeff{4};
            %                     u_coeff(j,:) = obj.SumCoeff({
            %                         conv(conv(conv(b,b),c),c);...
            %                         -4*conv(conv(conv(a,c),c),c);...
            %                         -4*conv(conv(conv(b,b),b),d);...
            %                         -27*conv(conv(conv(a,a),d),d);...
            %                         18*conv(conv(conv(a,b),c),d)
            %                         });
            %                 elseif  obj.obstacle.surfaceDeg(surf_ind) == 4
            %                     a = H_i_coeff{1}; b = H_i_coeff{2}; c = H_i_coeff{3}; d = H_i_coeff{4}; e = H_i_coeff{5};
            %                     u_coeff(j,:) = obj.SumCoeff({
            %                         256*conv(conv(conv(conv(conv(a,a),a),e),e),e);...
            %                         -192*conv(conv(conv(conv(conv(a,a),b),d),e),e);...
            %                         -128*conv(conv(conv(conv(conv(a,a),c),c),e),e);...
            %                         144*conv(conv(conv(conv(conv(a,a),c),d),d),e);...
            %                         -27*conv(conv(conv(conv(conv(a,a),d),d),d),d);...
            %                         144*conv(conv(conv(conv(conv(a,b),b),c),e),e);...
            %                         -6*conv(conv(conv(conv(conv(a,b),b),d),d),e);...
            %                         -80*conv(conv(conv(conv(conv(a,b),c),c),d),e);...
            %                         18*conv(conv(conv(conv(conv(a,b),c),d),d),d);...
            %                         16*conv(conv(conv(conv(conv(a,c),c),c),c),e);...
            %                         -4*conv(conv(conv(conv(conv(a,c),c),c),d),d);...
            %                         -27*conv(conv(conv(conv(conv(b,b),b),b),e),e);...
            %                         18*conv(conv(conv(conv(conv(b,b),b),c),d),e);...
            %                         -4*conv(conv(conv(conv(conv(b,b),b),d),d),d);...
            %                         -4*conv(conv(conv(conv(conv(b,b),c),c),c),e);...
            %                         1*conv(conv(conv(conv(conv(b,b),c),c),d),d)
            %                         });
            %                 else
            %                     CASPR_log.Error('Only surface degree 2 to 4 is supported in this algorithm');
            %                 end
            %
            %             end
            %             toc
            u_val = [];
            for i = 1:size(u_coeff_c,1)
                u_roots = roots(u_coeff_c(i,:));
                u_val = [u_val;u_roots];
                
            end
            
            [u_val,segment_ind] = obj.SortAnswerWithIndex(u_val,obj.u_range);
            
            if ~isempty(u_val)
                flag = 1;
            else
                flag = 0;
            end
            
        end
        
        %function to find intersection between OB_u and obstacle surface
        function [flag,u_val,val_seg_surf] = CurveSegment_Surface_Intersection(obj,model,q_begin,q_end,surf_ind,free_variable_index)
            v = []; val_seg_surf = [];
            flag = 0;u_val = [];
            %             possible_solution_number = repmat(obj.obstacle.surfaceDeg,obj.segment_num,1);
            %             possible_solution_number = possible_solution_number(:);
            %             surf_ind = repmat(1:obj.obstacle.surfaceNum,obj.segment_num,1);
            %             surf_ind = surf_ind(:);
            
            %             t_coeff = s_coeffs_T(P_end(:,1),P_end(:,2),P_end(:,3),P_begin(:,1),P_begin(:,2),P_begin(:,3),cell2mat(obj.obstacle.surfaceCoeffs));
            t_coeff = [];
            for i = 1:obj.segment_num
                t_coeff = s_coeffs_O(obj.OB_u{i},obj.obstacle.surfaceCoeffs{surf_ind});
                t_roots = roots(t_coeff);
                real_sol_ind = find(imag(t_roots) == 0);
                t_roots = t_roots(real_sol_ind);
                if ~isempty(t_roots)
                    u_val = [u_val;t_roots];  
                    val_seg_surf = [val_seg_surf;[t_roots, repmat([i,surf_ind],size(t_roots))]]; 
                end
            end
            
%             u_val = [];
%             sample_size =  obj.obstacle.surfaceDeg(surf_ind)*2 + 1;
%             
%             q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
%             u_sample = tan(q_sample/2);
%             q_update = q_begin;
%             for i = 1:size(q_sample,2)
%                 q_update(free_variable_index) = q_sample(i);
%                 [~,OB_i_u]  = obj.GetSegmentData(model,q_update);
%                 for j = 1:obj.segment_num
%                     f_sample(i,j) = obj.obstacle.surfaceEqu{surf_ind}(OB_i_u(j,1),OB_i_u(j,2),OB_i_u(j,3))*(1+u_sample(i)^2)^  obj.obstacle.surfaceDeg(surf_ind);
%                 end
%             end
%             for i = 1:obj.segment_num
%                 u_coeff(i,:) = GeneralMathOperations.PolynomialFit(u_sample', f_sample(:,i),   obj.obstacle.surfaceDeg(surf_ind)*2)';
%                 u_val = [u_val;roots(u_coeff(i,:))];
%             end
%             
%             [u_val,segment_ind] = obj.SortAnswerWithIndex(u_val,obj.u_range);
            
            
            if ~isempty(u_val)
                flag = 1;
            else
                flag = 0;
            end
        end
        
        %% function to find intersection between a line segment and the obstacle surface
        function [flag,v,val_seg_surf] = LineSegment_Surface_Intersection(obj,P_begin,P_end,surf_ind,need_varify)
            
            v = []; val_seg_surf = [];
            flag = 0;
            %             possible_solution_number = repmat(obj.obstacle.surfaceDeg,obj.segment_num,1);
            %             possible_solution_number = possible_solution_number(:);
            %             surf_ind = repmat(1:obj.obstacle.surfaceNum,obj.segment_num,1);
            %             surf_ind = surf_ind(:);
            
            %             t_coeff = s_coeffs_T(P_end(:,1),P_end(:,2),P_end(:,3),P_begin(:,1),P_begin(:,2),P_begin(:,3),cell2mat(obj.obstacle.surfaceCoeffs));
            t_coeff = [];
            for i = 1:obj.segment_num
                t_coeff = s_coeffs_T(P_end(i,:)',P_begin(i,:)',obj.obstacle.surfaceCoeffs{surf_ind});
                t_roots = roots(t_coeff);
                if ~isempty(t_roots)
                    v = [v;t_roots];  
                    val_seg_surf = [val_seg_surf;[t_roots, repmat([i,surf_ind],[obj.obstacle.surfaceDeg(surf_ind),1])]]; 
                end
            end
%             for i = 1:size(t_coeff,1)
%                 t_roots = roots(t_coeff(i,:));
%                 if ~isempty(t_roots)
%                     v_val = [v_val;t_roots];
%                 else
%                     
%                 end
%             end
%             Range = [0,1];
%             v_val = round(v_val,obj.ROUNDING_DIGIT);
%             real_sol_index = find(imag(v_val) == 0);
%             v_val = v_val(real_sol_index,:);
%             [v_val,ia,~] = unique(v_val,'rows');
%             real_sol_index = real_sol_index(ia,:);
%             remove_index = [find(v_val(:,1) < Range(1));find(v_val(:,1) > Range(2))];
%             real_sol_index(remove_index) = [];
%             v_val(remove_index,:) = [];
%             remove_ind = [];
            if ~isempty(v) && need_varify
                v(v>1) = []; v(v<0) = [];
                for i = 1:size(v,1)
                    for j = 1:obj.segment_num
                        intersection_pt(j,:) = round((P_end(j,:) - P_begin(j,:))*v(i) + P_begin(j,:),obj.ROUNDING_DIGIT);
                    end
                    %                     dfddf = scatter3(intersection_pt(:,1),intersection_pt(:,2),intersection_pt(:,3),'filled')
                    for j = 1:obj.obstacle.surfaceNum
                        tmp_in_surf = sign(obj.obstacle.surfaceEqu{j}(intersection_pt(:,1),intersection_pt(:,2),intersection_pt(:,3)));
                        tmp_in_surf(tmp_in_surf==0) = obj.obstacle.surfaceDirection(j);
                        in_surf(:,j) = tmp_in_surf;
                    end
                    for j = 1:obj.segment_num
                        if isequal(in_surf(j,:),obj.obstacle.surfaceDirection)
                            flag = 1;
                            return;                       
                        end
                    end
                    
                end
                flag = 0;
%                 v(logical(remove_ind),:) = [];
%                 flag = ~isempty(v);
                
            end
        end
        
        %% function to find intersection between boundary of implicit surface to cable surface
        function [flag,u_value,v_value,segment_ind] = SegmentIntersection_Gxyz(obj,bound_ind,OA_i_hat,is_dof_translation)
            u_value = [];segment_ind = [];
            v_value = []; P_j = [];
            t_roots = []; t = []; val_seg_bound = [];
            for i = 1:obj.segment_num
                t_coeffs = g_coeffs(obj.obstacle.boundaryEquCoeff{bound_ind},obj.G_coeffs(i,:)');
                t_roots = roots(t_coeffs);
                real_sol_ind = find(imag(t_roots) == 0);
                t_roots = t_roots(real_sol_ind);
                t_roots(t_roots > 1) = [];t_roots(t_roots < 0) = [];                
                 if ~isempty(t_roots)
                    t = [t;t_roots];  
                    for j = 1:size(t_roots,1)
                        P_j = [P_j,obj.obstacle.boundaryEqu{bound_ind}(t_roots(j))];
                    end
                    val_seg_bound = [val_seg_bound;[t_roots, repmat([i,bound_ind],size(t_roots))]]; 
                end
            end
            for i = 1:size(t,1)
                [u_val,v_val] = obj.Finduv(obj.OB_u{val_seg_bound(i,2)},OA_i_hat(val_seg_bound(i,2),:),P_j(:,i),is_dof_translation);
                if ~isempty(u_val) && u_val <= obj.u_range(2) &&  u_val >= obj.u_range(1) &&...
                        v_val <= 1 &&  v_val >= 0
                    u_value = [u_value;u_val];
                    v_value = [v_value;v_val];
                    segment_ind = [segment_ind;[u_val,val_seg_bound(i,2:end)]];
                end
            end
%             if is_dof_translation
%                 Gxyz_deg = 1;
%             else
%                 Gxyz_deg = 2;
%             end
%             
%             t_val = [];
%             for i = 1:obj.obstacle.boundaryNum
%                 t = linspace(0,1,obj.Boundary_equ_degree(i)*2 + 1);
%                 for j = 1:size(t,2)
%                     boundary_xyz(:,j) = obj.Boundary_equ{i}(t(j));
%                 end
%                 for j = 1:obj.segment_num
%                     cable_surf_sample(j,:) = obj.G_xyz{j}(boundary_xyz(1,:),boundary_xyz(2,:),boundary_xyz(3,:));
%                     cable_surf_t(j,:) = GeneralMathOperations.PolynomialFit(t', cable_surf_sample(j,:)', obj.Boundary_equ_degree(i)*Gxyz_deg)';
%                     t_roots = round(roots(cable_surf_t(j,:)),obj.ROUNDING_DIGIT);
%                     if ~isempty(t_roots)
%                         t_val = [t_val;t_roots];
%                     else
%                         t_val = [t_val;-1.*ones(size(cable_surf_t(j,:),2)-1,1)];
%                     end
%                 end
%                 
%                 [t_val,tmp_segment_ind] = obj.SortAnswerWithIndex(t_val,[0 1]);
%                 
%                 
%                 for j = 1:size(t_val,1)
%                     P_j = obj.Boundary_equ{i}(t_val(j));
%                     [u_val,v_val] = obj.Finduv(obj.OB_u{tmp_segment_ind(j)},OA_i_hat(tmp_segment_ind(j),:),P_j,is_dof_translation);
%                     if ~isempty(u_val) && u_val <= obj.u_range(2) &&  u_val >= obj.u_range(1)
%                         u_value = [u_value;u_val];
%                         v_value = [v_value;v_val];
%                         segment_ind = [segment_ind;tmp_segment_ind(j)];
%                     end
%                 end
%                 t_val = [];
%             end
            if ~isempty(u_value)
%                 remain_ind = v_value <= 1 & v_value >= 0;
%                 u_value = u_value(remain_ind);
%                 v_value = v_value(remain_ind);
%                 segment_ind = segment_ind(remain_ind);
                flag = 1;
            else
                flag = 0;
            end
            
        end
        %% function to check the intersected interval valid
        function has_intersected = IntervalVerify(obj,model,q1,q2)
            [OA_i,OB_i] = obj.GetSegmentData(model,(q1+q2)/2);
%             [a,b] = draw_robot(model,q1);
%             delete(a)
            for surf_ind = 1:obj.obstacle.surfaceNum
            [flag(surf_ind),~,~] = LineSegment_Surface_Intersection(obj,OA_i,OB_i,surf_ind,1);
            if any(flag)
                has_intersected = 1;
%                  delete(b)
                return;
            end
            end
             has_intersected = 0;
%               delete(b)
        end
        
        %% function to get the coeffcient of the quadra  surface
        function [OB_u,OB_u_denominator]  = GetOB_u(obj,model,q_begin,q_end,free_variable_index,is_dof_translation)
            if is_dof_translation
                sample_size = 2;
                q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
                u_sample = linspace(0,1,sample_size);
                H_i_u_denominator = ones(size(u_sample));
                OB_u_denominator = 1;
                OB_u_deg = 1;
            else
                sample_size = 3;
                q_sample = linspace(1.05*q_begin(free_variable_index),1.05*q_end(free_variable_index),sample_size);
                u_sample = tan(q_sample/2);
                H_i_u_denominator = (1+u_sample.^2);
                OB_u_denominator = @(u) 1+u.^2;
                OB_u_deg = 2;
            end
            q_update = q_begin;
            for i = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(i);
                [~,OB_i_u]  = obj.GetSegmentData(model,q_update);
                for j = 1:obj.segment_num
                    OB_i_u_sample{j}(i,:) = OB_i_u(j,:).*H_i_u_denominator(i);
                end
            end
            for i = 1:obj.segment_num
                for j = 1:3
                    OB_u{i}(j,:) = GeneralMathOperations.PolynomialFit(u_sample', OB_i_u_sample{i}(:,j), OB_u_deg)';
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
                for i=1:N %This is faster:
                    ui=(P_intersect-PA(i,:))*Si(i,:)'/(Si(i,:)*Si(i,:)');
                    distances(i)=norm(P_intersect-PA(i,:)-ui*Si(i,:));
                end
                %for i=1:N %http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html:
                %    distances(i) = norm(cross(P_intersect-PA(i,:),P_intersect-PB(i,:))) / norm(Si(i,:));
                %end
            end
        end
        
        function [cable_implicit_coeff,cable_implicit_fun] = GetGxyz(obj,model,q_begin,q_end,free_variable_index,is_dof_translation)
            cable_implicit_fun = [];cable_implicit_coeff = [];
            if is_dof_translation
                sample_size = 2;
            else
                sample_size = 7;
            end
            
            q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
            q_update = q_begin;
            surface_data = {};
            for i = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(i);
                [OA_i_u,OB_i_u]  = obj.GetSegmentData(model,q_update);
                t = [0.1;0.25;0.5;0.75;0.9];
                for j = 1:size(t,1)
                    tmp_data = (OB_i_u - OA_i_u).*t(j) + OA_i_u;
                    for k = 1:obj.segment_num
                        if i == 1 && j == 1
                            surface_data{k}(j,:) = tmp_data(k,:);
                        else
                            surface_data{k}(end+1,:) = tmp_data(k,:);
                        end
                    end
                end
            end
            v = zeros(obj.segment_num,10);
            for i = 1:obj.segment_num
                if is_dof_translation
                    v(i,7:end) = obj.Plane_fit(surface_data{i});
                    cable_implicit_fun{i} =@(x,y,z) v(i,1).*x +  v(i,2).*y +  v(i,3).*z +  v(i,4);
                else
                    [v(i,:),center(i,:)] = obj.Cone_fit(surface_data{i});
                    cable_implicit_fun{i} =@(x,y,z) v(i,1).*x.^2 + v(i,2).*y.^2 + v(i,3).*z.^2 + ...
                        v(i,4).*x.*y + v(i,5).*x.*z + v(i,6).*y.*z + v(i,7).*x + v(i,8).*y +...
                        v(i,9).*z + v(i,10);
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
        
        %% function to calculate the u,v where points on the G(u,v)
        function [u,v] = Finduv(obj,OB_u,OA,P,is_dof_translation)
            if is_dof_translation
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
        
        %% function to find line segment and surface intersection
        function t_c = t_coeffs(~,B1,B2,B3,A1,A2,A3,c)
            %T_COEFFS
            %    T_C = T_COEFFS(B1,B2,B3,A1,A2,A3,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16,C17,C18,C19,C20,C21,C22,C23,C24,C25,C26,C27,C28)
            
            %    This function was generated by the Symbolic Math Toolbox version 8.2.
            %    05-Oct-2020 18:17:25
            c1 = c(1,:);
            c2 = c(2,:);
            c3 = c(3,:);
            c4 = c(4,:);
            c5 = c(5,:);
            c6 = c(6,:);
            c7 = c(7,:);
            c8 = c(8,:);
            c9 = c(9,:);
            c10 = c(10,:);
            c11 = c(11,:);
            c12 = c(12,:);
            c13 = c(13,:);
            c14 = c(14,:);
            c15 = c(15,:);
            c16 = c(16,:);
            c17 = c(17,:);
            c18 = c(18,:);
            c19 = c(19,:);
            c20 = c(20,:);
            c21 = c(21,:);
            c22 = c(22,:);
            c23 = c(23,:);
            c24 = c(24,:);
            c25 = c(25,:);
            c26 = c(26,:);
            c27 = c(27,:);
            c28 = c(28,:);
            
            t2 = A1.^2;
            t3 = A1.^3;
            t4 = A2.^2;
            t5 = A2.^3;
            t6 = A3.^2;
            t7 = A3.^3;
            t8 = -B1;
            t9 = -B2;
            t10 = -B3;
            t11 = A1+t8;
            t12 = A2+t9;
            t13 = A3+t10;
            t14 = t11.^2;
            t15 = t11.^3;
            t16 = t12.^2;
            t17 = t12.^3;
            t18 = t13.^2;
            t19 = t13.^3;
            t4 =  c1.*t14.^2+c2.*t16.^2+c3.*t18.^2+c4.*t12.*t15+c5.*t13.*t15+c6.*t11.*t17+c7.*t13.*t17+c8.*t11.*t19+c9.*t12.*t19;
            t3 = -c10.*t15-c11.*t17-c12.*t19-A1.*c1.*t15.*4.0-A2.*c2.*t17.*4.0-A2.*c4.*t15-A3.*c5.*t15-A1.*c6.*t17-A3.*c3.*t19.*4.0-A3.*c7.*t17-A1.*c8.*t19-A2.*c9.*t19-c13.*t12.*t14-c14.*t13.*t14-c15.*t11.*t16-c16.*t13.*t16-c17.*t11.*t18-c18.*t12.*t18-A1.*c4.*t12.*t14.*3.0-A1.*c5.*t13.*t14.*3.0-A2.*c6.*t11.*t16.*3.0-A2.*c7.*t13.*t16.*3.0-A3.*c8.*t11.*t18.*3.0-A3.*c9.*t12.*t18.*3.0;
            t2 = c19.*t14+c20.*t16+c21.*t18+A1.*c10.*t14.*3.0+A2.*c11.*t16.*3.0+A2.*c13.*t14+A3.*c14.*t14+A1.*c15.*t16+A3.*c12.*t18.*3.0+A3.*c16.*t16+A1.*c17.*t18+A2.*c18.*t18+c1.*t2.*t14.*6.0+c2.*t4.*t16.*6.0+c3.*t6.*t18.*6.0+c22.*t11.*t12+c23.*t11.*t13+c24.*t12.*t13+c4.*t2.*t11.*t12.*3.0+c5.*t2.*t11.*t13.*3.0+c6.*t4.*t11.*t12.*3.0+c7.*t4.*t12.*t13.*3.0+c8.*t6.*t11.*t13.*3.0+c9.*t6.*t12.*t13.*3.0+A1.*A2.*c4.*t14.*3.0+A1.*A3.*c5.*t14.*3.0+A1.*A2.*c6.*t16.*3.0+A2.*A3.*c7.*t16.*3.0+A1.*A3.*c8.*t18.*3.0+A2.*A3.*c9.*t18.*3.0+A1.*c13.*t11.*t12.*2.0+A1.*c14.*t11.*t13.*2.0+A2.*c15.*t11.*t12.*2.0+A2.*c16.*t12.*t13.*2.0+A3.*c17.*t11.*t13.*2.0+A3.*c18.*t12.*t13.*2.0;
            t1 = -c25.*t11-c26.*t12-c27.*t13-A1.*c19.*t11.*2.0-A2.*c20.*t12.*2.0-A1.*c22.*t12-A2.*c22.*t11-A1.*c23.*t13-A3.*c21.*t13.*2.0-A3.*c23.*t11-A2.*c24.*t13-A3.*c24.*t12-c1.*t3.*t11.*4.0-c2.*t5.*t12.*4.0-c4.*t3.*t12-c5.*t3.*t13-c6.*t5.*t11-c3.*t7.*t13.*4.0-c10.*t2.*t11.*3.0-c7.*t5.*t13-c8.*t7.*t11-c11.*t4.*t12.*3.0-c13.*t2.*t12-c9.*t7.*t12-c14.*t2.*t13-c15.*t4.*t11-c12.*t6.*t13.*3.0-c16.*t4.*t13-c17.*t6.*t11-c18.*t6.*t12-A1.*A2.*c13.*t11.*2.0-A1.*A3.*c14.*t11.*2.0-A1.*A2.*c15.*t12.*2.0-A2.*A3.*c16.*t12.*2.0-A1.*A3.*c17.*t13.*2.0-A2.*A3.*c18.*t13.*2.0-A2.*c4.*t2.*t11.*3.0-A3.*c5.*t2.*t11.*3.0-A1.*c6.*t4.*t12.*3.0-A3.*c7.*t4.*t12.*3.0-A1.*c8.*t6.*t13.*3.0-A2.*c9.*t6.*t13.*3.0;
            t0 = c28+A1.*c25+A2.*c26+A3.*c27+c10.*t3+c11.*t5+c12.*t7+c19.*t2+c20.*t4+c21.*t6+c1.*t2.^2+c2.*t4.^2+c3.*t6.^2+A1.*A2.*c22+A1.*A3.*c23+A2.*A3.*c24+A2.*c4.*t3+A3.*c5.*t3+A1.*c6.*t5+A3.*c7.*t5+A1.*c8.*t7+A2.*c13.*t2+A2.*c9.*t7+A3.*c14.*t2+A1.*c15.*t4+A3.*c16.*t4+A1.*c17.*t6+A2.*c18.*t6;
            
            t_c = [t4(:),t3(:),t2(:),t1(:),t0(:)];
        end
        %% side functions to tidy up the prog
        function output = SumCoeff(~,P)
            output = P{1,:};
            for i = 2:size(P,1)
                output =  [zeros(1, size(P{i,:},2)-size(output,2)) output] + [zeros(1, size(output,2)-size(P{i,:},2)) P{i,:}];
            end
        end
        function [o1,o2] = SortAnswerWithIndex(obj,value,Range)
            o2 = [];
            val = round(value(:,1),obj.ROUNDING_DIGIT);             
            real_sol_index = find(imag(val) == 0);
            [uni_val,ia,~] = unique(val(real_sol_index),'rows');
            remove_index = [find(uni_val(:,1) < Range(1));find(uni_val(:,1) > Range(2))];
            uni_val(remove_index) = []; 
%             ia(remove_index) = [];
            for i = 1:size(uni_val,1)
                index = find(ismember(val,uni_val(i)));
                tmp_val = {uni_val(i),value(index,2:3)};
                o2 = [o2;tmp_val];
            end
            o1 = uni_val;
            
%             val(remove_index) = []; ia(remove_index) = [];
%             surf_ind = value(ia,3);
%             seg_ind = value(ia,2);
%             
%             segment_ind =  repmat(1:obj.segment_num,size(value,1)/obj.segment_num,1);
%             segment_ind = segment_ind(:);
%             ans_list = [value,segment_ind];
%             real_sol_index = find(imag(value) == 0);
%             ans_list = ans_list(real_sol_index,:);
%             [ans_list,ia,~] = unique(ans_list,'rows');
%             real_sol_index = real_sol_index(ia,:);
%             remove_index = [find(ans_list(:,1) < Range(1));find(ans_list(:,1) > Range(2))];
%             real_sol_index(remove_index) = [];
%             ans_list(remove_index,:) = [];
%             o1 = ans_list(:,1);
%             o2 = ans_list(:,2);
            
        end
        function out= findIntersection(~,first,second)
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

