% Class to compute whether a pose (dynamics) is within the interference
% free workspace (IFW)
%
% Author        : Paul Cheng
% Created       : 2020
% Description   :

classdef InterferenceFreeRayConditionCableObstacle < WorkspaceRayConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 5;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF;
    end
    
    properties (SetAccess = protected)
        % Set constants
        areDofsTranslation;         % Array for the q of the joint (true if translation and false if rotation)
        numDofs;                    % The number of dofs
        numCables;                  % The number of cables
        QuadSurf;                   % The obstacle surface equations
        surface_bound               % Obstacle boundary
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCableObstacle(model, min_ray_lengths, QuadSurf)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.QuadSurf = QuadSurf.implicit_equation;
            w.surface_bound = QuadSurf.boundary;
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            % Variable initialisation
            intervals = [];
            intervals_count = 1;
            syms u v t;
            free_variable_index = ws_ray.free_variable_index;
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            q_begin = [ws_ray.fixed_variables(1:free_variable_index-1);ws_ray.free_variable_range(1);ws_ray.fixed_variables(free_variable_index:end)];
            q_end = [ws_ray.fixed_variables(1:free_variable_index-1);ws_ray.free_variable_range(2);ws_ray.fixed_variables(free_variable_index:end)];
            all_intersection_poses =[q_begin,q_end];
%             clf
%             ob1 = fimplicit3(obj.QuadSurf,obj.surface_bound,'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.4);
%             hold on;intersected_pts = [];
%             model.update(q_begin,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
%             draw_robot(model);
%             model.update(q_end,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
%             draw_robot(model);
            % Att_pts{1} -> base point,  Att_pts{2}-> start point,  Att_pts{3}-> end point
                [Att_pts{1},Att_pts{2}] = obj.GetSegmentData(model,q_begin);
                [~,Att_pts{3}] = obj.GetSegmentData(model,q_end); 
            if is_dof_translation    
                
                for i = 1:obj.numCables
                    
%                     h(i)=patch('Faces',1:3,'Vertices',[Att_pts{3}(i,:);Att_pts{2}(i,:);Att_pts{1}(i,:)]);
%                     set(h(i),'FaceColor','r','EdgeColor','k','LineWidth',2,'FaceAlpha',0.5);
                    %% parametric form f(u,v) of the cable segment surface
                    parametric_cable_surf = (((Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*u +  Att_pts{2}(i,:)') - Att_pts{1}(i,:)').*v + Att_pts{1}(i,:)';
                    parametric_cable_surf_uv = @(u,v) (((Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*u +  Att_pts{2}(i,:)') - Att_pts{1}(i,:)').*v + Att_pts{1}(i,:)';
                    
                    [q_intersected,intersected_pts] = obj.ParametericSurfaceIntersection(parametric_cable_surf_uv,Att_pts{1}(i,:),q_begin,q_end);
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    intersected_pts = [intersected_pts,intersected_pts];
                    %% boundary curves intersection
                    Segment{1} =@(t) (Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*t + Att_pts{2}(i,:)';
                    Segment{2} =@(t) (Att_pts{3}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    Segment{3} =@(t) (Att_pts{2}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    [q_intersected,intersected_pts] = CurveSurfaceIntersection(obj,Segment,parametric_cable_surf_uv,Att_pts{1}(i,:)',q_begin,q_end);
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    intersected_pts = [intersected_pts,intersected_pts];
%                     if ~isempty(intersected_pts)
%                         inplot(i) = scatter3(intersected_pts(1,:),intersected_pts(2,:),intersected_pts(3,:));
%                     end
                    %                 delete(h(i));
                    %                 delete(inplot(i));
                end
                all_intersection_poses = unique(round(all_intersection_poses',obj.ROUNDING_DIGIT),'rows')';
                for i = 1:size(all_intersection_poses,2) - 1
                    has_intersected = obj.IntervalVerify(model,all_intersection_poses(:,i),all_intersection_poses(:,i+1));
                    if has_intersected == 0
                        intervals(intervals_count,:) = [all_intersection_poses(free_variable_index,i),all_intersection_poses(free_variable_index,i+1)];
                        intervals_count = intervals_count + 1;
                    end
                end
            else
                [R_coeff,T_coeff] = obj.RotationMatrixCoefficient(model,q_begin,q_end);
                [Si,base_point] = obj.GetSegmentEquation(model,R_coeff,T_coeff);
                C_D = [1 0 2 0 1];
                                
                for i = 1:obj.numCables
                    %% for plot only
%                     t1 = linspace(0,1,20);
%                     for ii = 1:size(t1,2)
%                         for iii = 1:3
%                             ee_point(iii,:) = polyval(Si{i}(iii,:),t1(ii)) / polyval(C_D,t1(ii));
%                         end
%                         LL = [base_point(:,i),ee_point + base_point(:,i)];
%                         incplot(i,ii) = plot3(LL(1,:),LL(2,:),LL(3,:),'k');
%                         hold on
%                         
%                     end
                    
                    Si_u =@(u) Si{i}*[u^4;u^3;u^2;u^1;1] / (C_D*[u^4;u^3;u^2;u^1;1]);
                    %% parametric form f(u,v) of the cable segment surface
                    parametric_cable_surf = Si_u(u)* v + base_point(:,i);
                    parametric_cable_surf_uv =@(u,v) Si_u(u)* v + base_point(:,i);
                    [q_intersected,intersected_pts] = obj.ParametericSurfaceIntersection(parametric_cable_surf_uv,base_point(:,i),q_begin,q_end);
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    intersected_pts = [intersected_pts,intersected_pts];
                    
                    %% boundary curves intersection
                    Segment{1} =@(t) Si_u(t) + base_point(:,i);
                    Segment{2} =@(t) (Att_pts{3}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    Segment{3} =@(t) (Att_pts{2}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    [q_intersected,intersected_pts] = CurveSurfaceIntersection(obj,Segment,parametric_cable_surf_uv,Att_pts{1}(i,:)',q_begin,q_end);
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    intersected_pts = [intersected_pts,intersected_pts];
%                     if ~isempty(intersected_pts)
%                         inplot(i) = scatter3(intersected_pts(1,:),intersected_pts(2,:),intersected_pts(3,:));
% %                         delete(inplot(i))    
%                     end
%                     delete(incplot)
%                     
                end  
                
                all_intersection_poses = unique(round(all_intersection_poses',obj.ROUNDING_DIGIT),'rows')';
                for i = 1:size(all_intersection_poses,2) - 1
                    has_intersected = obj.IntervalVerify(model,all_intersection_poses(:,i),all_intersection_poses(:,i+1));
                    if has_intersected == 0
                        intervals(intervals_count,:) = [all_intersection_poses(free_variable_index,i),all_intersection_poses(free_variable_index,i+1)];
                        intervals_count = intervals_count + 1;
                    end
                end
                
            end
            
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
        
        %% function to get the coefficient of rotational matrix and translational variables that represent the cable segment equ
        function [Rotation_Matrix_Coefficient,Translation_coeff] = RotationMatrixCoefficient(~,model,q_begin,q_end)
            % get start and end angles
            
            
            orientation_index = find(ismember(model.bodyModel.q_dofType,'ROTATION'));
            translation_index = find(ismember(model.bodyModel.q_dofType,'TRANSLATION'));
            Translation_coeff = q_begin(translation_index);
            if  ~round(norm(abs(q_begin(orientation_index)) - pi),4) && ...
                    ~round(norm(abs(q_end(orientation_index)) - pi),4)
                CASPR_log.Error('Start and end angle cannot be the same');
            end
            q_s = angle2quat(q_begin(orientation_index(1)),q_begin(orientation_index(2)),q_begin(orientation_index(3)),'XYZ');
            q_e = angle2quat(q_end(orientation_index(1)),q_end(orientation_index(2)),q_end(orientation_index(3)),'XYZ');
            
            %% sampling the matrix by 5 times since the max degree is 4
            
            t = linspace(0,1,5);
            
            for i = 1:length(t)
                q_t = quatinterp(q_s,q_e,t(i),'slerp');
                sample_R = quat2rotm(q_t);
                denominator_R(i) = polyval([1 0 2 0 1],t(i));
                numerator_R{i} = denominator_R(i) * sample_R;
            end
            for i = 1:9 % number of elements of rotation matrix
                for j = 1:5 % number of samples
                    sample_element(i,j) = numerator_R{j}(i);
                    M(j,:) = [t(j)^4 t(j)^3 t(j)^2 t(j) 1];
                end
                Rotation_Matrix_Coefficient(i,:) = M\sample_element(i,:)';
            end
            Rotation_Matrix_Coefficient = round(Rotation_Matrix_Coefficient,9);
            Rotation_Matrix_Coefficient(isnan(Rotation_Matrix_Coefficient))=0;
        end
        
        %% function to get the cable segment equation
        function [Si,base_point] = GetSegmentEquation(~,model,R,q)
            C_D = [1 0 2 0 1];
            for i = 1:model.numCables
                r_GA_i = model.cableModel.cables{1,i}.attachments{1,2}.r_GA';
                r_OA_i = model.cableModel.cables{1,i}.attachments{1,1}.r_OA';
                base_point(:,i) = r_OA_i;
                Si{i}(1,:) = SumCoeff({conv(q(1,:),C_D);(R(1,:)*r_GA_i(1) + R(2,:)*r_GA_i(2) + R(3,:)*r_GA_i(3));-C_D*r_OA_i(1)});
                Si{i}(2,:) = SumCoeff({conv(q(2,:),C_D);(R(4,:)*r_GA_i(1) + R(5,:)*r_GA_i(2) + R(6,:)*r_GA_i(3));-C_D*r_OA_i(2)});
                Si{i}(3,:) = SumCoeff({conv(q(3,:),C_D);(R(7,:)*r_GA_i(1) + R(8,:)*r_GA_i(2) + R(9,:)*r_GA_i(3));-C_D*r_OA_i(3)});
            end
        end
        
        %% function to calculate the intersected poses between quad-surface and cable segment bounded surface
        function [q_intersected,intersected_pts] = ParametericSurfaceIntersection(obj,uv_equ,base_att,q_begin,q_end)
            syms u v
            intersected_pts = [];q_intersected = [];
            para_equ = uv_equ(u,v);
            f_1 =@(u,v) obj.QuadSurf(para_equ(1),para_equ(2),para_equ(3));
            [v_coeff,~] = coeffs(f_1(u,v),v);
            % b^2 - 4ac
            f_2 = v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1);
            [u_numerator,~] = numden(f_2);
            [u_coeff,~] = coeffs(u_numerator,u);
            u_value = unique(double((roots(u_coeff))));
            u_value = u_value(imag(u_value)==0);
            u_value(u_value <0) = [];u_value(u_value > 1) = [];
            if ~isempty(u_value)
                for i = 1:size(u_value,1)
                    v_coeff = subs(v_coeff,u_value(i));
                    v_value(1) = double((-v_coeff(2) + sqrt(v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1)))/(2*v_coeff(1)));
                    v_value(2) = double((-v_coeff(2) + sqrt(v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1)))/(2*v_coeff(1)));
                    v_value = v_value(imag(v_value)==0);
                    v_value = unique(round(v_value,obj.ROUNDING_DIGIT));
                    v_value(v_value <0) = [];v_value(v_value > 1) = [];
                    %% find the corresponding pose by finding the same unit vector
                    if ~isempty(v_value)
                        for ii = 1:size(v_value,2)
                            %                         vec_1 = uv_equ(u,1) - base_att;
                            %                         vec_2 = uv_equ(u_value(i),1) - base_att;
                            %                         f_3 = (vec_1(1)^2 +  vec_1(2)^2 +  vec_1(3)^2)*vec_2(1)^2 ...
                            %                             /norm(vec_2)^2 - vec_1(1)^2;
                            %                         [u_1_numerator,~] = numden(f_3);
                            %                         [u_1_coeff,~] = coeffs(u_1_numerator,u);
                            %                         u_1_value = double(roots(u_1_coeff));
                            %                         u_1_value = u_1_value(imag(u_1_value)==0);
                            %                         u_1_value(u_1_value <0) = [];u_1_value(u_1_value > 1) = [];
                            %                         q_intersected = [q_intersected,(q_end - q_begin).*u_1_value' + q_begin];
                            %                         tmp_intersected_pts = [tmp_intersected_pts, parametric_cable_surf_uv(u_1_value,v_value)];
                            %% check if out of surface boundary
                            tmp_val = uv_equ(u_value(i),u_value(ii));
                            if tmp_val(1) <= obj.surface_bound(2) && tmp_val(1) >= obj.surface_bound(1) && ...
                                    tmp_val(2) <= obj.surface_bound(4) && tmp_val(2) >= obj.surface_bound(3) &&...
                                    tmp_val(3) <= obj.surface_bound(6) && tmp_val(3) >= obj.surface_bound(5)
                                q_intersected = [q_intersected,(q_end - q_begin)*u_value(i) + q_begin];
                                intersected_pts = [intersected_pts, uv_equ(u_value(i),u_value(ii))];
                            end
                            
                            %                                 q_intersected = [q_intersected,(q_end - q_begin)*u_value(i) + q_begin];
                            %                                 intersected_pts = [intersected_pts, uv_equ(u_value(i),u_value(ii))];
                            
                        end
                    end
                    %
                end
            end
        end
        
        %% function to calculate the intersected poses between quad-surface and 3 cable segment equations
        function [q_intersected,intersected_pts] = CurveSurfaceIntersection(obj,t_equ,uv_equ,base_att,q_begin,q_end)
            syms t u;
            q_intersected = [];intersected_pts = [];
            for i = 1:3
                %                     tmp_intersected_pts = [];
                para_equ = t_equ{i}(t);
                f_1 =@(t) obj.QuadSurf(para_equ(1),para_equ(2),para_equ(3));
                [t_numerator,t_denominator] = numden(f_1(t));
                [t_coeff,t_degree] = coeffs(t_numerator,t);
                t_ans = double(roots(t_coeff));
                t_ans = t_ans(imag(t_ans)==0);
                t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                %                     for ii = 1:size(t_ans,1)
                %                         tmp_intersected_pts = [tmp_intersected_pts,double(t_equ{i}(t_ans(i)))];
                %                     end
                if ~isempty(t_ans)
                    for ii = 1:size(t_ans,2)
                        %% finding the corresponding poses of intersection
                        %                         unit_vec_1 = uv_equ(u,1) - base_att;
                        %                         unit_vec_2 = tmp_intersected_pts(:,ii) - base_att;
                        %                         %                             u_value = double(solve((unit_vec_1(1)^2 +  unit_vec_1(2)^2 +  unit_vec_1(3)^2)*unit_vec_2(1)^2/norm(unit_vec_2)^2 - unit_vec_1(1)^2,'Real',true));
                        %                         u_value = double(solve(sqrt((unit_vec_1(1)^2 +  unit_vec_1(2)^2 +  unit_vec_1(3)^2))*unit_vec_2(1)/norm(unit_vec_2) - unit_vec_1(1),'Real',true));
                        %                         u_value(u_value <0) = [];u_value(u_value > 1) = [];
                        %                         if ~isempty(u_value)
                        %                             q_intersected = [q_intersected,(q_end - q_begin)*u_value + q_begin];
                        %                             intersected_pts = [intersected_pts,tmp_intersected_pts(:,ii)];
                        %                         end
                        
                        %% check if out of surface boundary
                        tmp_val = double(t_equ{i}(t_ans(ii)));
                        if tmp_val(1) <= obj.surface_bound(2) && tmp_val(1) >= obj.surface_bound(1) && ...
                                tmp_val(2) <= obj.surface_bound(4) && tmp_val(2) >= obj.surface_bound(3) &&...
                                tmp_val(3) <= obj.surface_bound(6) && tmp_val(3) >= obj.surface_bound(5)
                            intersected_pts = [intersected_pts, double(t_equ{i}(t_ans(ii)))];
                            q_intersected = [q_intersected,(q_end - q_begin)*t_ans(ii) + q_begin];
                        end
                    end
                end
            end
        end
        %% function to check the intersected interval valid
        function has_intersected = IntervalVerify(obj,model,q1,q2)
            syms t;
            [Att1,Att2] = obj.GetSegmentData(model,(q1+q2)/2);
            for i = 1:obj.numCables
                Check_seg_t =   (Att2(i,:) - Att1(i,:))'.*t + Att1(i,:)';
                f_1 =@(t) obj.QuadSurf(Check_seg_t(1),Check_seg_t(2),Check_seg_t(3));
                [t_coeff,~] = coeffs(f_1(t),t);
                t_ans = double(roots(t_coeff));
                t_ans = t_ans(imag(t_ans)==0);
                t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                if ~isempty(t_ans)
                    val(i) = 1;
                    has_intersected = 1;
                    return
                else
                    val(i) = 0;
                end
            end
            if  sum(val) == 0
                has_intersected = sum(val);
            end
        end
    end
end

function output = SumCoeff(P)
output = P{1,:};
for i = 2:size(P,1)
    output =  [zeros(1, size(P{i,:},2)-size(output,2)) output] + [zeros(1, size(output,2)-size(P{i,:},2)) P{i,:}];
end
end

