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
        
        MAX_DEGREE_TRANSLATION = 3;
        MAX_DEGREE_ORIENTATION = 6;
    end
    
    properties (SetAccess = protected)
        % Set constants
        areDofsTranslation;         % Array for the q of the joint (true if translation and false if rotation)
        numDofs;                    % The number of dofs
        numCables;                  % The number of cables
        degRedundancy;              % The degree of redundancy
        QuadSurf;                   % The obstacle surface equations
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCableObstacle(model, min_ray_lengths, QuadSurf)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.QuadSurf = QuadSurf;
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            interference_q = [];
            intervals =[];
            intervals_count = 1;
            % Variable initialisation
            syms u v t;
            q_zero = zeros(obj.numDofs, 1);
            free_variable_index = ws_ray.free_variable_index;
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            q_begin = [ws_ray.fixed_variables(1:free_variable_index-1);ws_ray.free_variable_range(1);ws_ray.fixed_variables(free_variable_index:end)];
            q_end = [ws_ray.fixed_variables(1:free_variable_index-1);ws_ray.free_variable_range(2);ws_ray.fixed_variables(free_variable_index:end)];
%             ob1 = fimplicit3(obj.QuadSurf,[-2 2 -2 2 0 2.5],'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.4)
%             hold on;
            if is_dof_translation
                % Att_pts{1} -> base point,  Att_pts{2}-> start point,  Att_pts{3}-> end point
                [Att_pts{1},Att_pts{2}] = obj.GetSegmentData(model,q_begin);
                [~,Att_pts{3}] = obj.GetSegmentData(model,q_end);
                
                
                i_plot = [];
                h = [];
                q_intersected = [];
                intersected_pts = [];
                tic
                for i = 1:obj.numCables
                    i_plot = [];
                    h=patch('Faces',1:3,'Vertices',[Att_pts{3}(i,:);Att_pts{2}(i,:);Att_pts{1}(i,:)]);
                    set(h,'FaceColor','r','EdgeColor','k','LineWidth',2,'FaceAlpha',0.5);
                    %% parametric form f(u,v) of the cable segment surface
                    parametric_cable_surf = (((Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*u +  Att_pts{2}(i,:)') - Att_pts{1}(i,:)').*v + Att_pts{1}(i,:)';
                    parametric_cable_surf_uv = @(u,v) (((Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*u +  Att_pts{2}(i,:)') - Att_pts{1}(i,:)').*v + Att_pts{1}(i,:)';
                    
                    %% boundary curves intersection
                    Segment{1} = (Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*t + Att_pts{2}(i,:)';
                    Segment{2} = (Att_pts{3}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    Segment{3} = (Att_pts{2}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    for ii = 1:3
                        tmp_intersected_pts = [];
                        f_1 =@(t) obj.QuadSurf(Segment{ii}(1),Segment{ii}(2),Segment{ii}(3));
                        %                         t_ans = double(solve(f_1(t),'Real',true));
                        [t_coeff,t_degree] = coeffs(f_1(t),t);
                        t_ans = roots(t_coeff);
                        t_ans = t_ans(imag(t_ans)==0);
                        t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                        for iii = 1:size(t_ans,1)
                            tmp_intersected_pts = [tmp_intersected_pts,double(subs(Segment{ii},t_ans(iii)))];
                        end
                        
                        for iii = 1:size(tmp_intersected_pts,2)
                            %                   angle(iii) = acos(((Att_pts{2}(i,:) - Att_pts{1}(i,:)) * (intersected_pts(:,iii)' - Att_pts{1}(i,:))')/...
                            %   (norm((Att_pts{2}(i,:)' - Att_pts{1}(i,:))) * norm((intersected_pts(:,iii) - Att_pts{1}(i,:)))));
                            %% finding the corresponding poses of intersection
                            unit_vec_1 = parametric_cable_surf_uv(u,1) - Att_pts{1}(i,:)';
                            unit_vec_2 = tmp_intersected_pts(:,iii) - Att_pts{1}(i,:)';
                            %                             u_value = double(solve((unit_vec_1(1)^2 +  unit_vec_1(2)^2 +  unit_vec_1(3)^2)*unit_vec_2(1)^2/norm(unit_vec_2)^2 - unit_vec_1(1)^2,'Real',true));
                            u_value = double(solve(sqrt((unit_vec_1(1)^2 +  unit_vec_1(2)^2 +  unit_vec_1(3)^2))*unit_vec_2(1)/norm(unit_vec_2) - unit_vec_1(1),'Real',true));
                            u_value(u_value <0) = [];u_value(u_value > 1) = [];
                            if ~isempty(u_value)
                                q_intersected = [q_intersected,(q_end - q_begin)*u_value + q_begin];
                                intersected_pts = [intersected_pts,tmp_intersected_pts(:,iii)];
                            end
                            
                        end
                        
                    end
                    %% surface inside the boundary intersection
                    f_1 =@(u,v) obj.QuadSurf(parametric_cable_surf(1),parametric_cable_surf(2),parametric_cable_surf(3));
                    [v_coeff,v_degree] = coeffs(f_1(u,v),v);
                    
                    f_2 = v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1);
                    [u_numerator,u_denominator] = numden(f_2);
                    [u_coeff,u_degree] = coeffs(u_numerator,u);
                    u_ans = unique(double((roots(u_coeff))));
                    u_ans = u_ans(imag(u_ans)==0);
                    u_ans(u_ans <0) = [];u_ans(u_ans > 1) = [];
                    if ~isempty(u_ans)
                        for ii = 1:size(u_ans,1)
                            %                         parametric_cable_surf_uv(u_ans(ii),v)
                            v_coeff = subs(v_coeff,u_ans(ii));
                            %                         v_ans = unqiue(double(roots(v_coeff)));
                            v_ans = unique(round(double(roots(v_coeff)),obj.ROUNDING_DIGIT));
                            v_ans(v_ans <0) = [];v_ans(v_ans > 1) = [];
                            
                            if ~isempty(v_ans)
                                unit_vec_1 = parametric_cable_surf_uv(u,1) - Att_pts{1}(i,:)';
                                unit_vec_2 = parametric_cable_surf_uv(u_ans(ii),1) - Att_pts{1}(i,:)';
                                u_value =  double(solve(sqrt((unit_vec_1(1)^2 +  unit_vec_1(2)^2 +  unit_vec_1(3)^2))*unit_vec_2(1)/norm(unit_vec_2) - unit_vec_1(1),'Real',true));
                                u_value(u_value <0) = [];u_value(u_value > 1) = [];
                                q_intersected = [q_intersected,(q_end - q_begin).*u_value' + q_begin];
                                
                                tmp_intersected_pts = [tmp_intersected_pts, parametric_cable_surf_uv(u_value,v_ans)];
                                if ~isempty(tmp_intersected_pts)
                                    i_plot(ii) = scatter3(tmp_intersected_pts(1,:),tmp_intersected_pts(2,:),tmp_intersected_pts(3,:));
                                    %
                                    %                     else
                                    %                         i_plot = [];
                                end
                            end
                        end
                    end
                    intersected_pts = [intersected_pts,tmp_intersected_pts(:,iii)];
                    
                    %                     i
                    if ~isempty(i_plot)
                        delete(i_plot)
                    end
                    %                     delete(h)
                end
                toc
                if ~isempty(q_intersected)
                    q_intersected = [q_intersected,q_begin,q_end];
                    q_intersected = unique(round(q_intersected',obj.ROUNDING_DIGIT),'rows')';
                    
                    for i = 1:size(q_intersected,2) - 1
                        check_interval = [];
                        model.update(q_intersected(:,i),zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1))
                        draw_robot(model)
                        model.update(q_intersected(:,i+1),zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1))
                        draw_robot(model)
                        [Att1,Att2] = obj.GetSegmentData(model,(q_intersected(:,i)+q_intersected(:,i+1))/2);
                        for ii = 1:obj.numCables
                            Check_seg_t =   (Att2(ii,:) - Att1(ii,:))'.*t + Att1(ii,:)';
                            f_1 =@(t) obj.QuadSurf(Check_seg_t(1),Check_seg_t(2),Check_seg_t(3));
                            [t_coeff,t_degree] = coeffs(f_1(t),t);
                            t_ans = double(roots(t_coeff));
                            t_ans = t_ans(imag(t_ans)==0);
                            t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                            if ~isempty(t_ans)
                                check_interval(ii) = 1;
                            else
                                check_interval(ii) = 0;
                            end
                        end
                        if all(check_interval == 0)
                            %% holding here, fill the codes later
                            intervals{intervals_count} = [q_intersected(:,i),q_intersected(:,i+1)];
                            intervals_count = intervals_count + 1;
                        end
                    end
                end
            else
                [R_coeff,T_coeff] = obj.RotationMatrixCoefficient(model,q_begin,q_end);
                [Si,base_point] = obj.GetSegmentEquation(model,R_coeff,T_coeff);
                C_D = [1 0 2 0 1];
                %% for plot only
                t1 = linspace(0,1,20);
                for i = 1:20
                    for ii = 1:8
                        for iii = 1:3
                            ee_point(iii,:) = polyval(Si{ii}(iii,:),t1(i)) / polyval(C_D,t1(i));
                        end
                        LL = [base_point(:,ii),ee_point + base_point(:,ii)];
                        plot3(LL(1,:),LL(2,:),LL(3,:),'k')
                        hold on
                    end
                end
                                
                for i = 1:obj.numCables
                    Si_u =@(u) Si{i}*[u^4;u^3;u^2;u^1;1] / (C_D*[u^4;u^3;u^2;u^1;1]);
                    
                    %% parametric form f(u,v) of the cable segment surface
                    parametric_cable_surf = Si_u(u)* v + base_point(:,i);
                    parametric_cable_surf_uv =@(u,v) Si_u(u)* v + base_point(:,i);
                    f_1 =@(u,v) obj.QuadSurf(parametric_cable_surf(1),parametric_cable_surf(2),parametric_cable_surf(3));
                    [v_coeff,v_degree] = coeffs(f_1(u,v),v);
                    
                    f_2 = v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1);
                    [u_numerator,u_denominator] = numden(f_2);
                    [u_coeff,u_degree] = coeffs(u_numerator,u);
                    u_ans = unique(double((roots(u_coeff))));
                    u_ans = u_ans(imag(u_ans)==0);
                    u_ans(u_ans <0) = [];u_ans(u_ans > 1) = [];
                    u_ans
                    u_ans;
                    
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
        
        %% function to get the coefficient of rotational matrix and translational variables
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
        
    end
end

function output = SumCoeff(P)
output = P{1,:};
for i = 2:size(P,1)
    output =  [zeros(1, size(P{i,:},2)-size(output,2)) output] + [zeros(1, size(output,2)-size(P{i,:},2)) P{i,:}];
end
end

