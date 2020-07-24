% Class to compute whether a pose (dynamics) is within the interference
% free workspace (IFW)
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
            %             syms u v t;
            %             syms x y z;
            
            free_variable_index = ws_ray.free_variable_index;
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            q_begin = [ws_ray.fixed_variables(1:free_variable_index-1);ws_ray.free_variable_range(1);ws_ray.fixed_variables(free_variable_index:end)];
            q_end = [ws_ray.fixed_variables(1:free_variable_index-1);ws_ray.free_variable_range(2);ws_ray.fixed_variables(free_variable_index:end)];
            all_intersection_poses = [q_begin,q_end];
            all_intersected_pts = [];
            % Att_pts{1} -> base point,  Att_pts{2}-> start point,  Att_pts{3}-> end point
            [Att_pts{1},Att_pts{2}] = obj.GetSegmentData(model,q_begin);
            [~,Att_pts{3}] = obj.GetSegmentData(model,q_end);
            
            QuadSurfCoeff  = GetQuadSurfCoeff(obj);
            
            if is_dof_translation
                
                for i = 1:obj.numCables
                    % parametric form f(u,v) of the cable segment surface
                    parametric_cable_surf_uv = @(u,v) (((Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*u +  Att_pts{2}(i,:)') - Att_pts{1}(i,:)').*v + Att_pts{1}(i,:)';
                    % three attachements from base frame
                    AttPts = [Att_pts{1}(i,:);Att_pts{2}(i,:);Att_pts{3}(i,:)];
                    % find the intersection of the triangle surface to the
                    % quadratic surface
                    [q_intersected,intersected_pts] = obj.ParametericSurfaceIntersectionTranslation(QuadSurfCoeff,AttPts,parametric_cable_surf_uv,q_begin,q_end);
%                     [q_intersected,intersected_pts] = obj.ParametericSurfaceIntersectionUniversal(parametric_cable_surf_uv,q_begin,q_end);
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    all_intersected_pts = [all_intersected_pts,intersected_pts];
                    
                    %% Three edges of the triangle surface
                    Segment{1} =@(t) (Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*t + Att_pts{2}(i,:)';
                    Segment{2} =@(t) (Att_pts{3}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    Segment{3} =@(t) (Att_pts{2}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    
                    % find the intersection of three edges with the
                    % quadratic surface
                    [q_intersected,intersected_pts] = Edges2QuadSurfIntersection(obj,QuadSurfCoeff,AttPts,Segment,parametric_cable_surf_uv,Att_pts{1}(i,:)',q_begin,q_end);
                    %  [q_intersected,intersected_pts] = Curve2QuadSurfIntersection(obj,Segment,parametric_cable_surf_uv,Att_pts{1}(i,:)',q_begin,q_end);
                    
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    all_intersected_pts = [all_intersected_pts,intersected_pts];
                    
                end
                
                all_intersection_poses = unique(round(all_intersection_poses',obj.ROUNDING_DIGIT),'rows')';
                
                %% verify the intersection interval
                pre_has_intersected = 1;
                for i = 1:size(all_intersection_poses,2) - 1
                    has_intersected = obj.IntervalVerify(model,QuadSurfCoeff,all_intersection_poses(:,i),all_intersection_poses(:,i+1));
                    if has_intersected == 0
                        if pre_has_intersected ~= 0
                            intervals(intervals_count,:) = [all_intersection_poses(free_variable_index,i),all_intersection_poses(free_variable_index,i+1)];
                            intervals_count = intervals_count + 1;
                        else
                            intervals(intervals_count-1 ,end) = all_intersection_poses(free_variable_index,i+1);
                        end
                    end
                    pre_has_intersected = has_intersected;
                end
            else
                [R_coeff,T_coeff,k_unit] = obj.RotationMatrixCoefficient(model,q_begin,q_end);
                [Si,base_point] = obj.GetSegmentEquation(model,R_coeff,T_coeff,k_unit);
                
                Common_Denominator = [k_unit^4 0 2*k_unit^2 0 1];
                
                for i = 1:obj.numCables
                    Si_u =@(u) Si{i}*[(u)^4;(u)^3;(u)^2;(u);1] / ((k_unit*u)^4 + 2*(u*k_unit)^2 + 1);
%                     Si_u =@(u) Si{i}*[u^4;u^3;u^2;u^1;1] / (Common_Denominator*[u^4;u^3;u^2;u^1;1]);
                    %% parametric form f(u,v) of the cable segment surface
                    parametric_cable_surf_uv =@(u,v) Si_u(u)* v + base_point(:,i);
                    
                    [q_intersected,intersected_pts] = obj.ParametericSurfaceIntersectionUniversal(parametric_cable_surf_uv,q_begin,q_end);
                    
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    all_intersected_pts = [all_intersected_pts,intersected_pts];
                    
                    %% boundary curves intersection
                    Segment{1} =@(t) Si_u(t) + base_point(:,i);
                    Segment{2} =@(t) (Att_pts{3}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    Segment{3} =@(t) (Att_pts{2}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    
                    rGAi = model.cableModel.cables{1,i}.attachments{1,2}.r_GA;
                    rOAi =  model.cableModel.cables{1,i}.attachments{1,2}.r_OA;
                    %                                      [q_intersected,intersected_pts] = Curve2QuadSurfIntersection(obj,Segment,parametric_cable_surf_uv,Att_pts{1}(i,:)',q_begin,q_end);
                    [q_intersected,intersected_pts] = Curve2QuadSurfIntersection(obj,Segment,q_begin,q_end,R_coeff,T_coeff,rGAi,QuadSurfCoeff);
                    
                    all_intersection_poses = [all_intersection_poses, q_intersected];
                    all_intersected_pts = [all_intersected_pts,intersected_pts];
                    
                end
                
                all_intersection_poses = unique(round(all_intersection_poses',obj.ROUNDING_DIGIT),'rows')';
                pre_has_intersected = 1;
                for i = 1:size(all_intersection_poses,2) - 1
                    has_intersected = obj.IntervalVerify(model,QuadSurfCoeff,all_intersection_poses(:,i),all_intersection_poses(:,i+1));
                    if has_intersected == 0
                        if pre_has_intersected ~= 0
                            intervals(intervals_count,:) = [all_intersection_poses(free_variable_index,i),all_intersection_poses(free_variable_index,i+1)];
                            intervals_count = intervals_count + 1;
                        else
                            intervals(intervals_count-1 ,end) = all_intersection_poses(free_variable_index,i+1);
                        end
                    end
                    pre_has_intersected = has_intersected;
                end
                
            end
            if ~isempty(all_intersected_pts)
            scatter3(all_intersected_pts(1,:),all_intersected_pts(2,:),all_intersected_pts(3,:),'filled');
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
        function [Rotation_Matrix_Coefficient,Translation_coeff,k_unit] = RotationMatrixCoefficient(~,model,q_begin,q_end)
            % get start and end angles
            
            orientation_index = find(ismember(model.bodyModel.q_dofType,'ROTATION'));
            translation_index = find(ismember(model.bodyModel.q_dofType,'TRANSLATION'));
            Translation_coeff = q_begin(translation_index);
            if  ~round(norm(abs(q_begin(orientation_index)) - pi),4) && ...
                    ~round(norm(abs(q_end(orientation_index)) - pi),4)
                CASPR_log.Error('Start and end angle cannot be the same');
            end
            q_s = quatnormalize(angle2quat(q_begin(orientation_index(1)),q_begin(orientation_index(2)),q_begin(orientation_index(3)),'XYZ'));
            q_e = quatnormalize(angle2quat(q_end(orientation_index(1)),q_end(orientation_index(2)),q_end(orientation_index(3)),'XYZ'));
            theta = acos(q_s*q_e');
            k_unit = tan(theta/2);
            %% sampling the matrix by 5 times since the max degree is 4
            
            t = linspace(0,1,5);
            if theta~=0
                tau = atan(k_unit*t)*2/theta;
            else
                tau = zeros(1,size(t,2));
            end
            for i = 1:length(t)
%                 q_t = quatinterp(q_s,q_e,t(i),'slerp');
                q_t = quatinterp(q_s,q_e,tau(i),'slerp');
                sample_R = quat2rotm(q_t);
%                 denominator_R(i) = polyval([1 0 2 0 1],t(i));
                denominator_R(i) = polyval([1 0 2 0 1],k_unit*t(i));
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
        function [Si,base_point] = GetSegmentEquation(~,model,R,q,k_unit)
            C_D = [k_unit^4 0 2*k_unit^2 0 1];
            
            for i = 1:model.numCables
                r_GA_i = model.cableModel.cables{1,i}.attachments{1,2}.r_GA';
                r_OA_i = model.cableModel.cables{1,i}.attachments{1,1}.r_OA';
                %                                 r_GA_i = sym('rGA%d',[3 1]);
                %                                 r_OA_i = sym('rOA%d',[3 1]);
                base_point(:,i) = r_OA_i;
                Si{i}(1,:) = SumCoeff({conv(q(1,:),C_D);(R(1,:)*r_GA_i(1) + R(2,:)*r_GA_i(2) + R(3,:)*r_GA_i(3));-C_D*r_OA_i(1)});
                Si{i}(2,:) = SumCoeff({conv(q(2,:),C_D);(R(4,:)*r_GA_i(1) + R(5,:)*r_GA_i(2) + R(6,:)*r_GA_i(3));-C_D*r_OA_i(2)});
                Si{i}(3,:) = SumCoeff({conv(q(3,:),C_D);(R(7,:)*r_GA_i(1) + R(8,:)*r_GA_i(2) + R(9,:)*r_GA_i(3));-C_D*r_OA_i(3)});
%                 Si{i}(1,:) = SumCoeff({q(1,:)*C_D;(R(1,:)*r_GA_i(1) + R(2,:)*r_GA_i(2) + R(3,:)*r_GA_i(3));-C_D*r_OA_i(1)});
%                 Si{i}(2,:) = SumCoeff({q(2,:)*C_D;(R(4,:)*r_GA_i(1) + R(5,:)*r_GA_i(2) + R(6,:)*r_GA_i(3));-C_D*r_OA_i(2)});
%                 Si{i}(3,:) = SumCoeff({q(3,:)*C_D;(R(7,:)*r_GA_i(1) + R(8,:)*r_GA_i(2) + R(9,:)*r_GA_i(3));-C_D*r_OA_i(3)});
%                 
                
            end
            
        end
        
        %% function to calculate the intersected poses between quad-surface and cable segment bounded surface
        function [q_intersected,intersected_pts] = ParametericSurfaceIntersectionUniversal(obj,uv_equ,q_begin,q_end)
            syms u v
            intersected_pts = [];q_intersected = [];
            para_equ = uv_equ(u,v);
            f_1 =@(u,v) obj.QuadSurf(para_equ(1),para_equ(2),para_equ(3));
            [v_coeff_u,~] = coeffs(f_1(u,v),v);
            % b^2 - 4ac
            f_2 = v_coeff_u(2)*v_coeff_u(2) - 4*v_coeff_u(3)*v_coeff_u(1);
            [u_numerator,u_denominator] = numden(f_2);
            [u_coeff,~] = coeffs(u_numerator,u);
            u_value = unique(double((roots(u_coeff))));
            u_value = u_value(imag(u_value)==0);
            u_value(u_value <0) = [];u_value(u_value > 1) = [];
            if ~isempty(u_value)
                for i = 1:size(u_value,1)
                    v_coeff = subs(v_coeff_u,u_value(i));
                    %                     v_value(1) = double((-v_coeff(2) + sqrt(v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1)))/(2*v_coeff(1)));
                    %                     v_value(2) = double((-v_coeff(2) - sqrt(v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1)))/(2*v_coeff(1)));
                    v_value = double(-v_coeff(2)/(2*v_coeff(1)));
                    
                    v_value = v_value(imag(v_value)==0);
                    
%                     v_value = unique(round(v_value,obj.ROUNDING_DIGIT));
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
                            tmp_val = uv_equ(u_value(i),v_value(ii));
                            if tmp_val(1) <= obj.surface_bound(2) && tmp_val(1) >= obj.surface_bound(1) && ...
                                    tmp_val(2) <= obj.surface_bound(4) && tmp_val(2) >= obj.surface_bound(3) &&...
                                    tmp_val(3) <= obj.surface_bound(6) && tmp_val(3) >= obj.surface_bound(5)
                                q_intersected = [q_intersected,(q_end - q_begin)*u_value(i) + q_begin];
                                intersected_pts = [intersected_pts, tmp_val];
                            end
                        end
                    end
                    %
                end
            end
        end
        %% function to calculate the intersected poses between quad-surface and 3 cable segment equations
        function [q_intersected,intersected_pts] = Curve2QuadSurfIntersection(obj,t_equ,q_begin,q_end,R_coeff,T_coeff,rGAi,QuadSurfCoeff)
            syms t u;
            s1 = QuadSurfCoeff(1);
            s2 = QuadSurfCoeff(2);
            s3 = QuadSurfCoeff(3);
            s4 = QuadSurfCoeff(4);
            s5 = QuadSurfCoeff(5);
            s6 = QuadSurfCoeff(6);
            s7 = QuadSurfCoeff(7);
            s8 = QuadSurfCoeff(8);
            s9 = QuadSurfCoeff(9);
            s10 = QuadSurfCoeff(10);
            
            rGA1 = rGAi(1);
            rGA2 = rGAi(2);
            rGA3 = rGAi(3);
            %             rOA1 = rOAi(1);
            %             rOA2 = rOAi(2);
            %             rOA3 = rOAi(3);
            
            
            r11 = R_coeff(1,1); r12 = R_coeff(1,2); r13 = R_coeff(1,3); r14 = R_coeff(1,4); r15 = R_coeff(1,5);
            r21 = R_coeff(2,1); r22 = R_coeff(2,2); r23 = R_coeff(2,3); r24 = R_coeff(2,4); r25 = R_coeff(2,5);
            r31 = R_coeff(3,1); r32 = R_coeff(3,2); r33 = R_coeff(3,3); r34 = R_coeff(3,4); r35 = R_coeff(3,5);
            r41 = R_coeff(4,1); r42 = R_coeff(4,2); r43 = R_coeff(4,3); r44 = R_coeff(4,4); r45 = R_coeff(4,5);
            r51 = R_coeff(5,1); r52 = R_coeff(5,2); r53 = R_coeff(5,3); r54 = R_coeff(5,4); r55 = R_coeff(5,5);
            r61 = R_coeff(6,1); r62 = R_coeff(6,2); r63 = R_coeff(6,3); r64 = R_coeff(6,4); r65 = R_coeff(6,5);
            r71 = R_coeff(7,1); r72 = R_coeff(7,2); r73 = R_coeff(7,3); r74 = R_coeff(7,4); r75 = R_coeff(7,5);
            r81 = R_coeff(8,1); r82 = R_coeff(8,2); r83 = R_coeff(8,3); r84 = R_coeff(8,4); r85 = R_coeff(8,5);
            r91 = R_coeff(9,1); r92 = R_coeff(9,2); r93 = R_coeff(9,3); r94 = R_coeff(9,4); r95 = R_coeff(9,5);
            
            q1 = T_coeff(1);q2 = T_coeff(2);q3 = T_coeff(3);
            
            q_intersected = [];intersected_pts = [];
            for i = 1:3
                %                     tmp_intersected_pts = [];
                para_equ = t_equ{i}(t);
                %                 f_1 =@(t) obj.QuadSurf(para_equ(1),para_equ(2),para_equ(3));
                %                 [t_numerator,t_denominator] = numden(f_1(t));
                %                 [t_coeff,t_degree] = coeffs(t_numerator,t);
                t_coeff = [
                    s1*q1^2 + s4*q1*q2 + s5*q1*q3 + 2*s1*q1*r11*rGA1 + 2*s1*q1*r21*rGA2 + 2*s1*q1*r31*rGA3 + s4*q1*r41*rGA1 + s4*q1*r51*rGA2 + s4*q1*r61*rGA3 + s5*q1*r71*rGA1 + s5*q1*r81*rGA2 + s5*q1*r91*rGA3 + s7*q1 + s2*q2^2 + s6*q2*q3 + s4*q2*r11*rGA1 + s4*q2*r21*rGA2 + s4*q2*r31*rGA3 + 2*s2*q2*r41*rGA1 + 2*s2*q2*r51*rGA2 + 2*s2*q2*r61*rGA3 + s6*q2*r71*rGA1 + s6*q2*r81*rGA2 + s6*q2*r91*rGA3 + s8*q2 + s3*q3^2 + s5*q3*r11*rGA1 + s5*q3*r21*rGA2 + s5*q3*r31*rGA3 + s6*q3*r41*rGA1 + s6*q3*r51*rGA2 + s6*q3*r61*rGA3 + 2*s3*q3*r71*rGA1 + 2*s3*q3*r81*rGA2 + 2*s3*q3*r91*rGA3 + s9*q3 + s1*r11^2*rGA1^2 + 2*s1*r11*r21*rGA1*rGA2 + 2*s1*r11*r31*rGA1*rGA3 + s4*r11*r41*rGA1^2 + s4*r11*r51*rGA1*rGA2 + s4*r11*r61*rGA1*rGA3 + s5*r11*r71*rGA1^2 + s5*r11*r81*rGA1*rGA2 + s5*r11*r91*rGA1*rGA3 + s7*r11*rGA1 + s1*r21^2*rGA2^2 + 2*s1*r21*r31*rGA2*rGA3 + s4*r21*r41*rGA1*rGA2 + s4*r21*r51*rGA2^2 + s4*r21*r61*rGA2*rGA3 + s5*r21*r71*rGA1*rGA2 + s5*r21*r81*rGA2^2 + s5*r21*r91*rGA2*rGA3 + s7*r21*rGA2 + s1*r31^2*rGA3^2 + s4*r31*r41*rGA1*rGA3 + s4*r31*r51*rGA2*rGA3 + s4*r31*r61*rGA3^2 + s5*r31*r71*rGA1*rGA3 + s5*r31*r81*rGA2*rGA3 + s5*r31*r91*rGA3^2 + s7*r31*rGA3 + s2*r41^2*rGA1^2 + 2*s2*r41*r51*rGA1*rGA2 + 2*s2*r41*r61*rGA1*rGA3 + s6*r41*r71*rGA1^2 + s6*r41*r81*rGA1*rGA2 + s6*r41*r91*rGA1*rGA3 + s8*r41*rGA1 + s2*r51^2*rGA2^2 + 2*s2*r51*r61*rGA2*rGA3 + s6*r51*r71*rGA1*rGA2 + s6*r51*r81*rGA2^2 + s6*r51*r91*rGA2*rGA3 + s8*r51*rGA2 + s2*r61^2*rGA3^2 + s6*r61*r71*rGA1*rGA3 + s6*r61*r81*rGA2*rGA3 + s6*r61*r91*rGA3^2 + s8*r61*rGA3 + s3*r71^2*rGA1^2 + 2*s3*r71*r81*rGA1*rGA2 + 2*s3*r71*r91*rGA1*rGA3 + s9*r71*rGA1 + s3*r81^2*rGA2^2 + 2*s3*r81*r91*rGA2*rGA3 + s9*r81*rGA2 + s3*r91^2*rGA3^2 + s9*r91*rGA3 + s10;
                    r12*rGA1*s7 + r22*rGA2*s7 + r32*rGA3*s7 + r42*rGA1*s8 + r52*rGA2*s8 + r62*rGA3*s8 + r72*rGA1*s9 + r82*rGA2*s9 + r92*rGA3*s9 + 2*q1*r12*rGA1*s1 + q2*r12*rGA1*s4 + q3*r12*rGA1*s5 + 2*q1*r22*rGA2*s1 + q2*r22*rGA2*s4 + q3*r22*rGA2*s5 + 2*q1*r32*rGA3*s1 + q2*r32*rGA3*s4 + q3*r32*rGA3*s5 + 2*q2*r42*rGA1*s2 + q1*r42*rGA1*s4 + q3*r42*rGA1*s6 + 2*q2*r52*rGA2*s2 + q1*r52*rGA2*s4 + q3*r52*rGA2*s6 + 2*q2*r62*rGA3*s2 + q1*r62*rGA3*s4 + q3*r62*rGA3*s6 + q1*r72*rGA1*s5 + 2*q3*r72*rGA1*s3 + q2*r72*rGA1*s6 + q1*r82*rGA2*s5 + 2*q3*r82*rGA2*s3 + q2*r82*rGA2*s6 + q1*r92*rGA3*s5 + 2*q3*r92*rGA3*s3 + q2*r92*rGA3*s6 + 2*r11*r12*rGA1^2*s1 + 2*r21*r22*rGA2^2*s1 + r11*r42*rGA1^2*s4 + r12*r41*rGA1^2*s4 + 2*r31*r32*rGA3^2*s1 + r21*r52*rGA2^2*s4 + r22*r51*rGA2^2*s4 + 2*r41*r42*rGA1^2*s2 + r11*r72*rGA1^2*s5 + r12*r71*rGA1^2*s5 + r31*r62*rGA3^2*s4 + r32*r61*rGA3^2*s4 + 2*r51*r52*rGA2^2*s2 + r21*r82*rGA2^2*s5 + r22*r81*rGA2^2*s5 + r41*r72*rGA1^2*s6 + r42*r71*rGA1^2*s6 + 2*r61*r62*rGA3^2*s2 + r31*r92*rGA3^2*s5 + r32*r91*rGA3^2*s5 + r51*r82*rGA2^2*s6 + r52*r81*rGA2^2*s6 + 2*r71*r72*rGA1^2*s3 + r61*r92*rGA3^2*s6 + r62*r91*rGA3^2*s6 + 2*r81*r82*rGA2^2*s3 + 2*r91*r92*rGA3^2*s3 + 2*r11*r22*rGA1*rGA2*s1 + 2*r12*r21*rGA1*rGA2*s1 + 2*r11*r32*rGA1*rGA3*s1 + 2*r12*r31*rGA1*rGA3*s1 + 2*r21*r32*rGA2*rGA3*s1 + 2*r22*r31*rGA2*rGA3*s1 + r11*r52*rGA1*rGA2*s4 + r12*r51*rGA1*rGA2*s4 + r21*r42*rGA1*rGA2*s4 + r22*r41*rGA1*rGA2*s4 + r11*r62*rGA1*rGA3*s4 + r12*r61*rGA1*rGA3*s4 + r31*r42*rGA1*rGA3*s4 + r32*r41*rGA1*rGA3*s4 + r21*r62*rGA2*rGA3*s4 + r22*r61*rGA2*rGA3*s4 + r31*r52*rGA2*rGA3*s4 + r32*r51*rGA2*rGA3*s4 + 2*r41*r52*rGA1*rGA2*s2 + 2*r42*r51*rGA1*rGA2*s2 + r11*r82*rGA1*rGA2*s5 + r12*r81*rGA1*rGA2*s5 + r21*r72*rGA1*rGA2*s5 + r22*r71*rGA1*rGA2*s5 + 2*r41*r62*rGA1*rGA3*s2 + 2*r42*r61*rGA1*rGA3*s2 + r11*r92*rGA1*rGA3*s5 + r12*r91*rGA1*rGA3*s5 + r31*r72*rGA1*rGA3*s5 + r32*r71*rGA1*rGA3*s5 + 2*r51*r62*rGA2*rGA3*s2 + 2*r52*r61*rGA2*rGA3*s2 + r21*r92*rGA2*rGA3*s5 + r22*r91*rGA2*rGA3*s5 + r31*r82*rGA2*rGA3*s5 + r32*r81*rGA2*rGA3*s5 + r41*r82*rGA1*rGA2*s6 + r42*r81*rGA1*rGA2*s6 + r51*r72*rGA1*rGA2*s6 + r52*r71*rGA1*rGA2*s6 + r41*r92*rGA1*rGA3*s6 + r42*r91*rGA1*rGA3*s6 + r61*r72*rGA1*rGA3*s6 + r62*r71*rGA1*rGA3*s6 + r51*r92*rGA2*rGA3*s6 + r52*r91*rGA2*rGA3*s6 + r61*r82*rGA2*rGA3*s6 + r62*r81*rGA2*rGA3*s6 + 2*r71*r82*rGA1*rGA2*s3 + 2*r72*r81*rGA1*rGA2*s3 + 2*r71*r92*rGA1*rGA3*s3 + 2*r72*r91*rGA1*rGA3*s3 + 2*r81*r92*rGA2*rGA3*s3 + 2*r82*r91*rGA2*rGA3*s3;
                    4*s10 + 4*q1*s7 + 4*q2*s8 + 4*q3*s9 + 4*q1^2*s1 + 4*q2^2*s2 + 4*q3^2*s3 + r12^2*rGA1^2*s1 + r22^2*rGA2^2*s1 + r32^2*rGA3^2*s1 + r42^2*rGA1^2*s2 + r52^2*rGA2^2*s2 + r62^2*rGA3^2*s2 + r72^2*rGA1^2*s3 + r82^2*rGA2^2*s3 + r92^2*rGA3^2*s3 + 4*q1*q2*s4 + 4*q1*q3*s5 + 4*q2*q3*s6 + 2*r11*rGA1*s7 + r13*rGA1*s7 + 2*r21*rGA2*s7 + r23*rGA2*s7 + 2*r31*rGA3*s7 + r33*rGA3*s7 + 2*r41*rGA1*s8 + r43*rGA1*s8 + 2*r51*rGA2*s8 + r53*rGA2*s8 + 2*r61*rGA3*s8 + r63*rGA3*s8 + 2*r71*rGA1*s9 + r73*rGA1*s9 + 2*r81*rGA2*s9 + r83*rGA2*s9 + 2*r91*rGA3*s9 + r93*rGA3*s9 + 4*q1*r11*rGA1*s1 + 2*q1*r13*rGA1*s1 + 2*q2*r11*rGA1*s4 + q2*r13*rGA1*s4 + 2*q3*r11*rGA1*s5 + q3*r13*rGA1*s5 + 4*q1*r21*rGA2*s1 + 2*q1*r23*rGA2*s1 + 2*q2*r21*rGA2*s4 + q2*r23*rGA2*s4 + 2*q3*r21*rGA2*s5 + q3*r23*rGA2*s5 + 4*q1*r31*rGA3*s1 + 2*q1*r33*rGA3*s1 + 2*q2*r31*rGA3*s4 + q2*r33*rGA3*s4 + 2*q3*r31*rGA3*s5 + q3*r33*rGA3*s5 + 4*q2*r41*rGA1*s2 + 2*q1*r41*rGA1*s4 + 2*q2*r43*rGA1*s2 + q1*r43*rGA1*s4 + 2*q3*r41*rGA1*s6 + q3*r43*rGA1*s6 + 4*q2*r51*rGA2*s2 + 2*q1*r51*rGA2*s4 + 2*q2*r53*rGA2*s2 + q1*r53*rGA2*s4 + 2*q3*r51*rGA2*s6 + q3*r53*rGA2*s6 + 4*q2*r61*rGA3*s2 + 2*q1*r61*rGA3*s4 + 2*q2*r63*rGA3*s2 + q1*r63*rGA3*s4 + 2*q3*r61*rGA3*s6 + q3*r63*rGA3*s6 + 2*q1*r71*rGA1*s5 + 4*q3*r71*rGA1*s3 + q1*r73*rGA1*s5 + 2*q2*r71*rGA1*s6 + 2*q3*r73*rGA1*s3 + q2*r73*rGA1*s6 + 2*q1*r81*rGA2*s5 + 4*q3*r81*rGA2*s3 + q1*r83*rGA2*s5 + 2*q2*r81*rGA2*s6 + 2*q3*r83*rGA2*s3 + q2*r83*rGA2*s6 + 2*q1*r91*rGA3*s5 + 4*q3*r91*rGA3*s3 + q1*r93*rGA3*s5 + 2*q2*r91*rGA3*s6 + 2*q3*r93*rGA3*s3 + q2*r93*rGA3*s6 + 2*r11*r13*rGA1^2*s1 + 2*r21*r23*rGA2^2*s1 + r11*r43*rGA1^2*s4 + r12*r42*rGA1^2*s4 + r13*r41*rGA1^2*s4 + 2*r31*r33*rGA3^2*s1 + r21*r53*rGA2^2*s4 + r22*r52*rGA2^2*s4 + r23*r51*rGA2^2*s4 + 2*r41*r43*rGA1^2*s2 + r11*r73*rGA1^2*s5 + r12*r72*rGA1^2*s5 + r13*r71*rGA1^2*s5 + r31*r63*rGA3^2*s4 + r32*r62*rGA3^2*s4 + r33*r61*rGA3^2*s4 + 2*r51*r53*rGA2^2*s2 + r21*r83*rGA2^2*s5 + r22*r82*rGA2^2*s5 + r23*r81*rGA2^2*s5 + r41*r73*rGA1^2*s6 + r42*r72*rGA1^2*s6 + r43*r71*rGA1^2*s6 + 2*r61*r63*rGA3^2*s2 + r31*r93*rGA3^2*s5 + r32*r92*rGA3^2*s5 + r33*r91*rGA3^2*s5 + r51*r83*rGA2^2*s6 + r52*r82*rGA2^2*s6 + r53*r81*rGA2^2*s6 + 2*r71*r73*rGA1^2*s3 + r61*r93*rGA3^2*s6 + r62*r92*rGA3^2*s6 + r63*r91*rGA3^2*s6 + 2*r81*r83*rGA2^2*s3 + 2*r91*r93*rGA3^2*s3 + 2*r11*r23*rGA1*rGA2*s1 + 2*r12*r22*rGA1*rGA2*s1 + 2*r13*r21*rGA1*rGA2*s1 + 2*r11*r33*rGA1*rGA3*s1 + 2*r12*r32*rGA1*rGA3*s1 + 2*r13*r31*rGA1*rGA3*s1 + 2*r21*r33*rGA2*rGA3*s1 + 2*r22*r32*rGA2*rGA3*s1 + 2*r23*r31*rGA2*rGA3*s1 + r11*r53*rGA1*rGA2*s4 + r12*r52*rGA1*rGA2*s4 + r13*r51*rGA1*rGA2*s4 + r21*r43*rGA1*rGA2*s4 + r22*r42*rGA1*rGA2*s4 + r23*r41*rGA1*rGA2*s4 + r11*r63*rGA1*rGA3*s4 + r12*r62*rGA1*rGA3*s4 + r13*r61*rGA1*rGA3*s4 + r31*r43*rGA1*rGA3*s4 + r32*r42*rGA1*rGA3*s4 + r33*r41*rGA1*rGA3*s4 + r21*r63*rGA2*rGA3*s4 + r22*r62*rGA2*rGA3*s4 + r23*r61*rGA2*rGA3*s4 + r31*r53*rGA2*rGA3*s4 + r32*r52*rGA2*rGA3*s4 + r33*r51*rGA2*rGA3*s4 + 2*r41*r53*rGA1*rGA2*s2 + 2*r42*r52*rGA1*rGA2*s2 + 2*r43*r51*rGA1*rGA2*s2 + r11*r83*rGA1*rGA2*s5 + r12*r82*rGA1*rGA2*s5 + r13*r81*rGA1*rGA2*s5 + r21*r73*rGA1*rGA2*s5 + r22*r72*rGA1*rGA2*s5 + r23*r71*rGA1*rGA2*s5 + 2*r41*r63*rGA1*rGA3*s2 + 2*r42*r62*rGA1*rGA3*s2 + 2*r43*r61*rGA1*rGA3*s2 + r11*r93*rGA1*rGA3*s5 + r12*r92*rGA1*rGA3*s5 + r13*r91*rGA1*rGA3*s5 + r31*r73*rGA1*rGA3*s5 + r32*r72*rGA1*rGA3*s5 + r33*r71*rGA1*rGA3*s5 + 2*r51*r63*rGA2*rGA3*s2 + 2*r52*r62*rGA2*rGA3*s2 + 2*r53*r61*rGA2*rGA3*s2 + r21*r93*rGA2*rGA3*s5 + r22*r92*rGA2*rGA3*s5 + r23*r91*rGA2*rGA3*s5 + r31*r83*rGA2*rGA3*s5 + r32*r82*rGA2*rGA3*s5 + r33*r81*rGA2*rGA3*s5 + r41*r83*rGA1*rGA2*s6 + r42*r82*rGA1*rGA2*s6 + r43*r81*rGA1*rGA2*s6 + r51*r73*rGA1*rGA2*s6 + r52*r72*rGA1*rGA2*s6 + r53*r71*rGA1*rGA2*s6 + r41*r93*rGA1*rGA3*s6 + r42*r92*rGA1*rGA3*s6 + r43*r91*rGA1*rGA3*s6 + r61*r73*rGA1*rGA3*s6 + r62*r72*rGA1*rGA3*s6 + r63*r71*rGA1*rGA3*s6 + r51*r93*rGA2*rGA3*s6 + r52*r92*rGA2*rGA3*s6 + r53*r91*rGA2*rGA3*s6 + r61*r83*rGA2*rGA3*s6 + r62*r82*rGA2*rGA3*s6 + r63*r81*rGA2*rGA3*s6 + 2*r71*r83*rGA1*rGA2*s3 + 2*r72*r82*rGA1*rGA2*s3 + 2*r73*r81*rGA1*rGA2*s3 + 2*r71*r93*rGA1*rGA3*s3 + 2*r72*r92*rGA1*rGA3*s3 + 2*r73*r91*rGA1*rGA3*s3 + 2*r81*r93*rGA2*rGA3*s3 + 2*r82*r92*rGA2*rGA3*s3 + 2*r83*r91*rGA2*rGA3*s3;
                    ];
                t_ans(1) = (-t_coeff(2) + sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                t_ans(2) = (-t_coeff(2) - sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                
                t_ans = t_ans(imag(t_ans)==0);
                t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                
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
        
        %% close form solution for translation
        function [q_intersected,intersected_pts] = ParametericSurfaceIntersectionTranslation(obj,QuadSurfCoeff,AttPts,uv_equ,q_begin,q_end)
            intersected_pts = [];q_intersected = [];
            
            a11 = AttPts(1,1);
            a12 = AttPts(1,2);
            a13 = AttPts(1,3);
            a21 = AttPts(2,1);
            a22 = AttPts(2,2);
            a23 = AttPts(2,3);
            a31 = AttPts(3,1);
            a32 = AttPts(3,2);
            a33 = AttPts(3,3);
            
            s1 = QuadSurfCoeff(1);
            s2 = QuadSurfCoeff(2);
            s3 = QuadSurfCoeff(3);
            s4 = QuadSurfCoeff(4);
            s5 = QuadSurfCoeff(5);
            s6 = QuadSurfCoeff(6);
            s7 = QuadSurfCoeff(7);
            s8 = QuadSurfCoeff(8);
            s9 = QuadSurfCoeff(9);
            s10 = QuadSurfCoeff(10);
            
            v_coeff_u =@(u) [s1*(a11 - a21 + u*(a21 - a31))^2 + s2*(a12 - a22 + u*(a22 - a32))^2 + s3*(a13 - a23 + u*(a23 - a33))^2 + s4*(a11 - a21 + u*(a21 - a31))*(a12 - a22 + u*(a22 - a32)) + s5*(a11 - a21 + u*(a21 - a31))*(a13 - a23 + u*(a23 - a33)) + s6*(a12 - a22 + u*(a22 - a32))*(a13 - a23 + u*(a23 - a33));
                - s7*(a11 - a21 + u*(a21 - a31)) - s8*(a12 - a22 + u*(a22 - a32)) - s9*(a13 - a23 + u*(a23 - a33)) - 2*a11*s1*(a11 - a21 + u*(a21 - a31)) - a12*s4*(a11 - a21 + u*(a21 - a31)) - 2*a12*s2*(a12 - a22 + u*(a22 - a32)) - a13*s5*(a11 - a21 + u*(a21 - a31)) - a11*s4*(a12 - a22 + u*(a22 - a32)) - a13*s6*(a12 - a22 + u*(a22 - a32)) - a11*s5*(a13 - a23 + u*(a23 - a33)) - 2*a13*s3*(a13 - a23 + u*(a23 - a33)) - a12*s6*(a13 - a23 + u*(a23 - a33));
                s1*a11^2 + s4*a11*a12 + s5*a11*a13 + s7*a11 + s2*a12^2 + s6*a12*a13 + s8*a12 + s3*a13^2 + s9*a13 + s10;];
            
            u_coeff = [a11^2*a22^2*s4^2 - 4*s1*s2*a11^2*a22^2 + 2*a11^2*a22*a23*s4*s5 - 4*s1*a11^2*a22*a23*s6 - 2*a11^2*a22*a32*s4^2 + 8*s1*s2*a11^2*a22*a32 - 2*a11^2*a22*a33*s4*s5 + 4*s1*a11^2*a22*a33*s6 + a11^2*a23^2*s5^2 - 4*s1*s3*a11^2*a23^2 - 2*a11^2*a23*a32*s4*s5 + 4*s1*a11^2*a23*a32*s6 - 2*a11^2*a23*a33*s5^2 + 8*s1*s3*a11^2*a23*a33 + a11^2*a32^2*s4^2 - 4*s1*s2*a11^2*a32^2 + 2*a11^2*a32*a33*s4*s5 - 4*s1*a11^2*a32*a33*s6 + a11^2*a33^2*s5^2 - 4*s1*s3*a11^2*a33^2 - 2*a11*a12*a21*a22*s4^2 + 8*s1*s2*a11*a12*a21*a22 - 2*a11*a12*a21*a23*s4*s5 + 4*s1*a11*a12*a21*a23*s6 + 2*a11*a12*a21*a32*s4^2 - 8*s1*s2*a11*a12*a21*a32 + 2*a11*a12*a21*a33*s4*s5 - 4*s1*a11*a12*a21*a33*s6 - 2*a11*a12*a22*a23*s4*s6 + 4*s2*a11*a12*a22*a23*s5 + 2*a11*a12*a22*a31*s4^2 - 8*s1*s2*a11*a12*a22*a31 + 2*a11*a12*a22*a33*s4*s6 - 4*s2*a11*a12*a22*a33*s5 - 4*s3*a11*a12*a23^2*s4 + 2*a11*a12*a23^2*s5*s6 + 2*a11*a12*a23*a31*s4*s5 - 4*s1*a11*a12*a23*a31*s6 + 2*a11*a12*a23*a32*s4*s6 - 4*s2*a11*a12*a23*a32*s5 + 8*s3*a11*a12*a23*a33*s4 - 4*a11*a12*a23*a33*s5*s6 - 2*a11*a12*a31*a32*s4^2 + 8*s1*s2*a11*a12*a31*a32 - 2*a11*a12*a31*a33*s4*s5 + 4*s1*a11*a12*a31*a33*s6 - 2*a11*a12*a32*a33*s4*s6 + 4*s2*a11*a12*a32*a33*s5 - 4*s3*a11*a12*a33^2*s4 + 2*a11*a12*a33^2*s5*s6 - 2*a11*a13*a21*a22*s4*s5 + 4*s1*a11*a13*a21*a22*s6 - 2*a11*a13*a21*a23*s5^2 + 8*s1*s3*a11*a13*a21*a23 + 2*a11*a13*a21*a32*s4*s5 - 4*s1*a11*a13*a21*a32*s6 + 2*a11*a13*a21*a33*s5^2 - 8*s1*s3*a11*a13*a21*a33 + 2*a11*a13*a22^2*s4*s6 - 4*s2*a11*a13*a22^2*s5 + 4*s3*a11*a13*a22*a23*s4 - 2*a11*a13*a22*a23*s5*s6 + 2*a11*a13*a22*a31*s4*s5 - 4*s1*a11*a13*a22*a31*s6 - 4*a11*a13*a22*a32*s4*s6 + 8*s2*a11*a13*a22*a32*s5 - 4*s3*a11*a13*a22*a33*s4 + 2*a11*a13*a22*a33*s5*s6 + 2*a11*a13*a23*a31*s5^2 - 8*s1*s3*a11*a13*a23*a31 - 4*s3*a11*a13*a23*a32*s4 + 2*a11*a13*a23*a32*s5*s6 - 2*a11*a13*a31*a32*s4*s5 + 4*s1*a11*a13*a31*a32*s6 - 2*a11*a13*a31*a33*s5^2 + 8*s1*s3*a11*a13*a31*a33 + 2*a11*a13*a32^2*s4*s6 - 4*s2*a11*a13*a32^2*s5 + 4*s3*a11*a13*a32*a33*s4 - 2*a11*a13*a32*a33*s5*s6 - 2*a11*a21*a22*s4*s7 + 4*s1*a11*a21*a22*s8 - 2*a11*a21*a23*s5*s7 + 4*s1*a11*a21*a23*s9 + 2*a11*a21*a32*s4*s7 - 4*s1*a11*a21*a32*s8 + 2*a11*a21*a33*s5*s7 - 4*s1*a11*a21*a33*s9 + 2*a11*a22^2*s4*s8 - 4*s2*a11*a22^2*s7 + 2*a11*a22*a23*s4*s9 + 2*a11*a22*a23*s5*s8 - 4*a11*a22*a23*s6*s7 + 2*a11*a22*a31*s4*s7 - 4*s1*a11*a22*a31*s8 - 4*a11*a22*a32*s4*s8 + 8*s2*a11*a22*a32*s7 - 2*a11*a22*a33*s4*s9 - 2*a11*a22*a33*s5*s8 + 4*a11*a22*a33*s6*s7 + 2*a11*a23^2*s5*s9 - 4*s3*a11*a23^2*s7 + 2*a11*a23*a31*s5*s7 - 4*s1*a11*a23*a31*s9 - 2*a11*a23*a32*s4*s9 - 2*a11*a23*a32*s5*s8 + 4*a11*a23*a32*s6*s7 - 4*a11*a23*a33*s5*s9 + 8*s3*a11*a23*a33*s7 - 2*a11*a31*a32*s4*s7 + 4*s1*a11*a31*a32*s8 - 2*a11*a31*a33*s5*s7 + 4*s1*a11*a31*a33*s9 + 2*a11*a32^2*s4*s8 - 4*s2*a11*a32^2*s7 + 2*a11*a32*a33*s4*s9 + 2*a11*a32*a33*s5*s8 - 4*a11*a32*a33*s6*s7 + 2*a11*a33^2*s5*s9 - 4*s3*a11*a33^2*s7 + a12^2*a21^2*s4^2 - 4*s1*s2*a12^2*a21^2 + 2*a12^2*a21*a23*s4*s6 - 4*s2*a12^2*a21*a23*s5 - 2*a12^2*a21*a31*s4^2 + 8*s1*s2*a12^2*a21*a31 - 2*a12^2*a21*a33*s4*s6 + 4*s2*a12^2*a21*a33*s5 + a12^2*a23^2*s6^2 - 4*s2*s3*a12^2*a23^2 - 2*a12^2*a23*a31*s4*s6 + 4*s2*a12^2*a23*a31*s5 - 2*a12^2*a23*a33*s6^2 + 8*s2*s3*a12^2*a23*a33 + a12^2*a31^2*s4^2 - 4*s1*s2*a12^2*a31^2 + 2*a12^2*a31*a33*s4*s6 - 4*s2*a12^2*a31*a33*s5 + a12^2*a33^2*s6^2 - 4*s2*s3*a12^2*a33^2 + 2*a12*a13*a21^2*s4*s5 - 4*s1*a12*a13*a21^2*s6 - 2*a12*a13*a21*a22*s4*s6 + 4*s2*a12*a13*a21*a22*s5 + 4*s3*a12*a13*a21*a23*s4 - 2*a12*a13*a21*a23*s5*s6 - 4*a12*a13*a21*a31*s4*s5 + 8*s1*a12*a13*a21*a31*s6 + 2*a12*a13*a21*a32*s4*s6 - 4*s2*a12*a13*a21*a32*s5 - 4*s3*a12*a13*a21*a33*s4 + 2*a12*a13*a21*a33*s5*s6 - 2*a12*a13*a22*a23*s6^2 + 8*s2*s3*a12*a13*a22*a23 + 2*a12*a13*a22*a31*s4*s6 - 4*s2*a12*a13*a22*a31*s5 + 2*a12*a13*a22*a33*s6^2 - 8*s2*s3*a12*a13*a22*a33 - 4*s3*a12*a13*a23*a31*s4 + 2*a12*a13*a23*a31*s5*s6 + 2*a12*a13*a23*a32*s6^2 - 8*s2*s3*a12*a13*a23*a32 + 2*a12*a13*a31^2*s4*s5 - 4*s1*a12*a13*a31^2*s6 - 2*a12*a13*a31*a32*s4*s6 + 4*s2*a12*a13*a31*a32*s5 + 4*s3*a12*a13*a31*a33*s4 - 2*a12*a13*a31*a33*s5*s6 - 2*a12*a13*a32*a33*s6^2 + 8*s2*s3*a12*a13*a32*a33 + 2*a12*a21^2*s4*s7 - 4*s1*a12*a21^2*s8 - 2*a12*a21*a22*s4*s8 + 4*s2*a12*a21*a22*s7 + 2*a12*a21*a23*s4*s9 - 4*a12*a21*a23*s5*s8 + 2*a12*a21*a23*s6*s7 - 4*a12*a21*a31*s4*s7 + 8*s1*a12*a21*a31*s8 + 2*a12*a21*a32*s4*s8 - 4*s2*a12*a21*a32*s7 - 2*a12*a21*a33*s4*s9 + 4*a12*a21*a33*s5*s8 - 2*a12*a21*a33*s6*s7 - 2*a12*a22*a23*s6*s8 + 4*s2*a12*a22*a23*s9 + 2*a12*a22*a31*s4*s8 - 4*s2*a12*a22*a31*s7 + 2*a12*a22*a33*s6*s8 - 4*s2*a12*a22*a33*s9 + 2*a12*a23^2*s6*s9 - 4*s3*a12*a23^2*s8 - 2*a12*a23*a31*s4*s9 + 4*a12*a23*a31*s5*s8 - 2*a12*a23*a31*s6*s7 + 2*a12*a23*a32*s6*s8 - 4*s2*a12*a23*a32*s9 - 4*a12*a23*a33*s6*s9 + 8*s3*a12*a23*a33*s8 + 2*a12*a31^2*s4*s7 - 4*s1*a12*a31^2*s8 - 2*a12*a31*a32*s4*s8 + 4*s2*a12*a31*a32*s7 + 2*a12*a31*a33*s4*s9 - 4*a12*a31*a33*s5*s8 + 2*a12*a31*a33*s6*s7 - 2*a12*a32*a33*s6*s8 + 4*s2*a12*a32*a33*s9 + 2*a12*a33^2*s6*s9 - 4*s3*a12*a33^2*s8 + a13^2*a21^2*s5^2 - 4*s1*s3*a13^2*a21^2 - 4*s3*a13^2*a21*a22*s4 + 2*a13^2*a21*a22*s5*s6 - 2*a13^2*a21*a31*s5^2 + 8*s1*s3*a13^2*a21*a31 + 4*s3*a13^2*a21*a32*s4 - 2*a13^2*a21*a32*s5*s6 + a13^2*a22^2*s6^2 - 4*s2*s3*a13^2*a22^2 + 4*s3*a13^2*a22*a31*s4 - 2*a13^2*a22*a31*s5*s6 - 2*a13^2*a22*a32*s6^2 + 8*s2*s3*a13^2*a22*a32 + a13^2*a31^2*s5^2 - 4*s1*s3*a13^2*a31^2 - 4*s3*a13^2*a31*a32*s4 + 2*a13^2*a31*a32*s5*s6 + a13^2*a32^2*s6^2 - 4*s2*s3*a13^2*a32^2 + 2*a13*a21^2*s5*s7 - 4*s1*a13*a21^2*s9 - 4*a13*a21*a22*s4*s9 + 2*a13*a21*a22*s5*s8 + 2*a13*a21*a22*s6*s7 - 2*a13*a21*a23*s5*s9 + 4*s3*a13*a21*a23*s7 - 4*a13*a21*a31*s5*s7 + 8*s1*a13*a21*a31*s9 + 4*a13*a21*a32*s4*s9 - 2*a13*a21*a32*s5*s8 - 2*a13*a21*a32*s6*s7 + 2*a13*a21*a33*s5*s9 - 4*s3*a13*a21*a33*s7 + 2*a13*a22^2*s6*s8 - 4*s2*a13*a22^2*s9 - 2*a13*a22*a23*s6*s9 + 4*s3*a13*a22*a23*s8 + 4*a13*a22*a31*s4*s9 - 2*a13*a22*a31*s5*s8 - 2*a13*a22*a31*s6*s7 - 4*a13*a22*a32*s6*s8 + 8*s2*a13*a22*a32*s9 + 2*a13*a22*a33*s6*s9 - 4*s3*a13*a22*a33*s8 + 2*a13*a23*a31*s5*s9 - 4*s3*a13*a23*a31*s7 + 2*a13*a23*a32*s6*s9 - 4*s3*a13*a23*a32*s8 + 2*a13*a31^2*s5*s7 - 4*s1*a13*a31^2*s9 - 4*a13*a31*a32*s4*s9 + 2*a13*a31*a32*s5*s8 + 2*a13*a31*a32*s6*s7 - 2*a13*a31*a33*s5*s9 + 4*s3*a13*a31*a33*s7 + 2*a13*a32^2*s6*s8 - 4*s2*a13*a32^2*s9 - 2*a13*a32*a33*s6*s9 + 4*s3*a13*a32*a33*s8 + a21^2*s7^2 - 4*s1*s10*a21^2 - 4*s10*a21*a22*s4 + 2*a21*a22*s7*s8 - 4*s10*a21*a23*s5 + 2*a21*a23*s7*s9 - 2*a21*a31*s7^2 + 8*s1*s10*a21*a31 + 4*s10*a21*a32*s4 - 2*a21*a32*s7*s8 + 4*s10*a21*a33*s5 - 2*a21*a33*s7*s9 + a22^2*s8^2 - 4*s2*s10*a22^2 - 4*s10*a22*a23*s6 + 2*a22*a23*s8*s9 + 4*s10*a22*a31*s4 - 2*a22*a31*s7*s8 - 2*a22*a32*s8^2 + 8*s2*s10*a22*a32 + 4*s10*a22*a33*s6 - 2*a22*a33*s8*s9 + a23^2*s9^2 - 4*s3*s10*a23^2 + 4*s10*a23*a31*s5 - 2*a23*a31*s7*s9 + 4*s10*a23*a32*s6 - 2*a23*a32*s8*s9 - 2*a23*a33*s9^2 + 8*s3*s10*a23*a33 + a31^2*s7^2 - 4*s1*s10*a31^2 - 4*s10*a31*a32*s4 + 2*a31*a32*s7*s8 - 4*s10*a31*a33*s5 + 2*a31*a33*s7*s9 + a32^2*s8^2 - 4*s2*s10*a32^2 - 4*s10*a32*a33*s6 + 2*a32*a33*s8*s9 + a33^2*s9^2 - 4*s3*s10*a33^2;
                - 2*a11^2*a22^2*s4^2 + 8*s1*s2*a11^2*a22^2 - 4*a11^2*a22*a23*s4*s5 + 8*s1*a11^2*a22*a23*s6 + 2*a32*a11^2*a22*s4^2 + 2*a33*a11^2*a22*s4*s5 - 2*a11^2*a22*s4*s7 - 4*a33*s1*a11^2*a22*s6 + 4*s1*a11^2*a22*s8 - 8*a32*s1*s2*a11^2*a22 - 2*a11^2*a23^2*s5^2 + 8*s1*s3*a11^2*a23^2 + 2*a32*a11^2*a23*s4*s5 + 2*a33*a11^2*a23*s5^2 - 2*a11^2*a23*s5*s7 - 4*a32*s1*a11^2*a23*s6 + 4*s1*a11^2*a23*s9 - 8*a33*s1*s3*a11^2*a23 + 2*a32*a11^2*s4*s7 + 2*a33*a11^2*s5*s7 - 4*a32*s1*a11^2*s8 - 4*a33*s1*a11^2*s9 + 4*a11*a12*a21*a22*s4^2 - 16*s1*s2*a11*a12*a21*a22 + 4*a11*a12*a21*a23*s4*s5 - 8*s1*a11*a12*a21*a23*s6 - 2*a32*a11*a12*a21*s4^2 - 2*a33*a11*a12*a21*s4*s5 + 2*a11*a12*a21*s4*s7 + 4*a33*s1*a11*a12*a21*s6 - 4*s1*a11*a12*a21*s8 + 8*a32*s1*s2*a11*a12*a21 + 4*a11*a12*a22*a23*s4*s6 - 8*s2*a11*a12*a22*a23*s5 - 2*a31*a11*a12*a22*s4^2 - 2*a33*a11*a12*a22*s4*s6 + 2*a11*a12*a22*s4*s8 + 4*a33*s2*a11*a12*a22*s5 - 4*s2*a11*a12*a22*s7 + 8*a31*s1*s2*a11*a12*a22 + 8*s3*a11*a12*a23^2*s4 - 4*a11*a12*a23^2*s5*s6 - 2*a31*a11*a12*a23*s4*s5 - 2*a32*a11*a12*a23*s4*s6 + 4*a11*a12*a23*s4*s9 - 8*a33*s3*a11*a12*a23*s4 + 4*a33*a11*a12*a23*s5*s6 - 2*a11*a12*a23*s5*s8 + 4*a32*s2*a11*a12*a23*s5 - 2*a11*a12*a23*s6*s7 + 4*a31*s1*a11*a12*a23*s6 - 2*a31*a11*a12*s4*s7 - 2*a32*a11*a12*s4*s8 - 4*a33*a11*a12*s4*s9 + 2*a33*a11*a12*s5*s8 + 2*a33*a11*a12*s6*s7 + 4*a32*s2*a11*a12*s7 + 4*a31*s1*a11*a12*s8 + 4*a11*a13*a21*a22*s4*s5 - 8*s1*a11*a13*a21*a22*s6 + 4*a11*a13*a21*a23*s5^2 - 16*s1*s3*a11*a13*a21*a23 - 2*a32*a11*a13*a21*s4*s5 - 2*a33*a11*a13*a21*s5^2 + 2*a11*a13*a21*s5*s7 + 4*a32*s1*a11*a13*a21*s6 - 4*s1*a11*a13*a21*s9 + 8*a33*s1*s3*a11*a13*a21 - 4*a11*a13*a22^2*s4*s6 + 8*s2*a11*a13*a22^2*s5 - 8*s3*a11*a13*a22*a23*s4 + 4*a11*a13*a22*a23*s5*s6 - 2*a31*a11*a13*a22*s4*s5 + 4*a32*a11*a13*a22*s4*s6 - 2*a11*a13*a22*s4*s9 + 4*a33*s3*a11*a13*a22*s4 - 2*a33*a11*a13*a22*s5*s6 + 4*a11*a13*a22*s5*s8 - 8*a32*s2*a11*a13*a22*s5 - 2*a11*a13*a22*s6*s7 + 4*a31*s1*a11*a13*a22*s6 + 4*a32*s3*a11*a13*a23*s4 - 2*a31*a11*a13*a23*s5^2 - 2*a32*a11*a13*a23*s5*s6 + 2*a11*a13*a23*s5*s9 - 4*s3*a11*a13*a23*s7 + 8*a31*s1*s3*a11*a13*a23 + 2*a32*a11*a13*s4*s9 - 2*a31*a11*a13*s5*s7 - 4*a32*a11*a13*s5*s8 - 2*a33*a11*a13*s5*s9 + 2*a32*a11*a13*s6*s7 + 4*a33*s3*a11*a13*s7 + 4*a31*s1*a11*a13*s9 + 4*a11*a21*a22*s4*s7 - 8*s1*a11*a21*a22*s8 + 4*a11*a21*a23*s5*s7 - 8*s1*a11*a21*a23*s9 - 2*a32*a11*a21*s4*s7 - 2*a33*a11*a21*s5*s7 + 2*a11*a21*s7^2 + 4*a32*s1*a11*a21*s8 + 4*a33*s1*a11*a21*s9 - 8*s1*s10*a11*a21 - 4*a11*a22^2*s4*s8 + 8*s2*a11*a22^2*s7 - 4*a11*a22*a23*s4*s9 - 4*a11*a22*a23*s5*s8 + 8*a11*a22*a23*s6*s7 - 2*a31*a11*a22*s4*s7 + 4*a32*a11*a22*s4*s8 + 2*a33*a11*a22*s4*s9 - 4*s10*a11*a22*s4 + 2*a33*a11*a22*s5*s8 - 4*a33*a11*a22*s6*s7 + 2*a11*a22*s7*s8 - 8*a32*s2*a11*a22*s7 + 4*a31*s1*a11*a22*s8 - 4*a11*a23^2*s5*s9 + 8*s3*a11*a23^2*s7 + 2*a32*a11*a23*s4*s9 - 2*a31*a11*a23*s5*s7 + 2*a32*a11*a23*s5*s8 + 4*a33*a11*a23*s5*s9 - 4*s10*a11*a23*s5 - 4*a32*a11*a23*s6*s7 + 2*a11*a23*s7*s9 - 8*a33*s3*a11*a23*s7 + 4*a31*s1*a11*a23*s9 + 4*a32*s10*a11*s4 + 4*a33*s10*a11*s5 - 2*a31*a11*s7^2 - 2*a32*a11*s7*s8 - 2*a33*a11*s7*s9 + 8*a31*s1*s10*a11 - 2*a12^2*a21^2*s4^2 + 8*s1*s2*a12^2*a21^2 - 4*a12^2*a21*a23*s4*s6 + 8*s2*a12^2*a21*a23*s5 + 2*a31*a12^2*a21*s4^2 + 2*a33*a12^2*a21*s4*s6 - 2*a12^2*a21*s4*s8 - 4*a33*s2*a12^2*a21*s5 + 4*s2*a12^2*a21*s7 - 8*a31*s1*s2*a12^2*a21 - 2*a12^2*a23^2*s6^2 + 8*s2*s3*a12^2*a23^2 + 2*a31*a12^2*a23*s4*s6 - 4*a31*s2*a12^2*a23*s5 + 2*a33*a12^2*a23*s6^2 - 2*a12^2*a23*s6*s8 + 4*s2*a12^2*a23*s9 - 8*a33*s2*s3*a12^2*a23 + 2*a31*a12^2*s4*s8 + 2*a33*a12^2*s6*s8 - 4*a31*s2*a12^2*s7 - 4*a33*s2*a12^2*s9 - 4*a12*a13*a21^2*s4*s5 + 8*s1*a12*a13*a21^2*s6 + 4*a12*a13*a21*a22*s4*s6 - 8*s2*a12*a13*a21*a22*s5 - 8*s3*a12*a13*a21*a23*s4 + 4*a12*a13*a21*a23*s5*s6 + 4*a31*a12*a13*a21*s4*s5 - 2*a32*a12*a13*a21*s4*s6 - 2*a12*a13*a21*s4*s9 + 4*a33*s3*a12*a13*a21*s4 - 2*a33*a12*a13*a21*s5*s6 - 2*a12*a13*a21*s5*s8 + 4*a32*s2*a12*a13*a21*s5 + 4*a12*a13*a21*s6*s7 - 8*a31*s1*a12*a13*a21*s6 + 4*a12*a13*a22*a23*s6^2 - 16*s2*s3*a12*a13*a22*a23 - 2*a31*a12*a13*a22*s4*s6 + 4*a31*s2*a12*a13*a22*s5 - 2*a33*a12*a13*a22*s6^2 + 2*a12*a13*a22*s6*s8 - 4*s2*a12*a13*a22*s9 + 8*a33*s2*s3*a12*a13*a22 + 4*a31*s3*a12*a13*a23*s4 - 2*a31*a12*a13*a23*s5*s6 - 2*a32*a12*a13*a23*s6^2 + 2*a12*a13*a23*s6*s9 - 4*s3*a12*a13*a23*s8 + 8*a32*s2*s3*a12*a13*a23 + 2*a31*a12*a13*s4*s9 + 2*a31*a12*a13*s5*s8 - 4*a31*a12*a13*s6*s7 - 2*a32*a12*a13*s6*s8 - 2*a33*a12*a13*s6*s9 + 4*a33*s3*a12*a13*s8 + 4*a32*s2*a12*a13*s9 - 4*a12*a21^2*s4*s7 + 8*s1*a12*a21^2*s8 + 4*a12*a21*a22*s4*s8 - 8*s2*a12*a21*a22*s7 - 4*a12*a21*a23*s4*s9 + 8*a12*a21*a23*s5*s8 - 4*a12*a21*a23*s6*s7 + 4*a31*a12*a21*s4*s7 - 2*a32*a12*a21*s4*s8 + 2*a33*a12*a21*s4*s9 - 4*s10*a12*a21*s4 - 4*a33*a12*a21*s5*s8 + 2*a33*a12*a21*s6*s7 + 2*a12*a21*s7*s8 + 4*a32*s2*a12*a21*s7 - 8*a31*s1*a12*a21*s8 + 4*a12*a22*a23*s6*s8 - 8*s2*a12*a22*a23*s9 - 2*a31*a12*a22*s4*s8 - 2*a33*a12*a22*s6*s8 + 4*a31*s2*a12*a22*s7 + 2*a12*a22*s8^2 + 4*a33*s2*a12*a22*s9 - 8*s2*s10*a12*a22 - 4*a12*a23^2*s6*s9 + 8*s3*a12*a23^2*s8 + 2*a31*a12*a23*s4*s9 - 4*a31*a12*a23*s5*s8 + 2*a31*a12*a23*s6*s7 - 2*a32*a12*a23*s6*s8 + 4*a33*a12*a23*s6*s9 - 4*s10*a12*a23*s6 + 2*a12*a23*s8*s9 - 8*a33*s3*a12*a23*s8 + 4*a32*s2*a12*a23*s9 + 4*a31*s10*a12*s4 + 4*a33*s10*a12*s6 - 2*a31*a12*s7*s8 - 2*a32*a12*s8^2 - 2*a33*a12*s8*s9 + 8*a32*s2*s10*a12 - 2*a13^2*a21^2*s5^2 + 8*s1*s3*a13^2*a21^2 + 8*s3*a13^2*a21*a22*s4 - 4*a13^2*a21*a22*s5*s6 - 4*a32*s3*a13^2*a21*s4 + 2*a31*a13^2*a21*s5^2 + 2*a32*a13^2*a21*s5*s6 - 2*a13^2*a21*s5*s9 + 4*s3*a13^2*a21*s7 - 8*a31*s1*s3*a13^2*a21 - 2*a13^2*a22^2*s6^2 + 8*s2*s3*a13^2*a22^2 - 4*a31*s3*a13^2*a22*s4 + 2*a31*a13^2*a22*s5*s6 + 2*a32*a13^2*a22*s6^2 - 2*a13^2*a22*s6*s9 + 4*s3*a13^2*a22*s8 - 8*a32*s2*s3*a13^2*a22 + 2*a31*a13^2*s5*s9 + 2*a32*a13^2*s6*s9 - 4*a31*s3*a13^2*s7 - 4*a32*s3*a13^2*s8 - 4*a13*a21^2*s5*s7 + 8*s1*a13*a21^2*s9 + 8*a13*a21*a22*s4*s9 - 4*a13*a21*a22*s5*s8 - 4*a13*a21*a22*s6*s7 + 4*a13*a21*a23*s5*s9 - 8*s3*a13*a21*a23*s7 - 4*a32*a13*a21*s4*s9 + 4*a31*a13*a21*s5*s7 + 2*a32*a13*a21*s5*s8 - 2*a33*a13*a21*s5*s9 - 4*s10*a13*a21*s5 + 2*a32*a13*a21*s6*s7 + 2*a13*a21*s7*s9 + 4*a33*s3*a13*a21*s7 - 8*a31*s1*a13*a21*s9 - 4*a13*a22^2*s6*s8 + 8*s2*a13*a22^2*s9 + 4*a13*a22*a23*s6*s9 - 8*s3*a13*a22*a23*s8 - 4*a31*a13*a22*s4*s9 + 2*a31*a13*a22*s5*s8 + 2*a31*a13*a22*s6*s7 + 4*a32*a13*a22*s6*s8 - 2*a33*a13*a22*s6*s9 - 4*s10*a13*a22*s6 + 2*a13*a22*s8*s9 + 4*a33*s3*a13*a22*s8 - 8*a32*s2*a13*a22*s9 - 2*a31*a13*a23*s5*s9 - 2*a32*a13*a23*s6*s9 + 4*a31*s3*a13*a23*s7 + 4*a32*s3*a13*a23*s8 + 2*a13*a23*s9^2 - 8*s3*s10*a13*a23 + 4*a31*s10*a13*s5 + 4*a32*s10*a13*s6 - 2*a31*a13*s7*s9 - 2*a32*a13*s8*s9 - 2*a33*a13*s9^2 + 8*a33*s3*s10*a13 - 2*a21^2*s7^2 + 8*s1*s10*a21^2 + 8*s10*a21*a22*s4 - 4*a21*a22*s7*s8 + 8*s10*a21*a23*s5 - 4*a21*a23*s7*s9 - 4*a32*s10*a21*s4 - 4*a33*s10*a21*s5 + 2*a31*a21*s7^2 + 2*a32*a21*s7*s8 + 2*a33*a21*s7*s9 - 8*a31*s1*s10*a21 - 2*a22^2*s8^2 + 8*s2*s10*a22^2 + 8*s10*a22*a23*s6 - 4*a22*a23*s8*s9 - 4*a31*s10*a22*s4 - 4*a33*s10*a22*s6 + 2*a31*a22*s7*s8 + 2*a32*a22*s8^2 + 2*a33*a22*s8*s9 - 8*a32*s2*s10*a22 - 2*a23^2*s9^2 + 8*s3*s10*a23^2 - 4*a31*s10*a23*s5 - 4*a32*s10*a23*s6 + 2*a31*a23*s7*s9 + 2*a32*a23*s8*s9 + 2*a33*a23*s9^2 - 8*a33*s3*s10*a23;
                a11^2*a22^2*s4^2 - 4*s1*s2*a11^2*a22^2 + 2*a11^2*a22*a23*s4*s5 - 4*s1*a11^2*a22*a23*s6 + 2*a11^2*a22*s4*s7 - 4*s1*a11^2*a22*s8 + a11^2*a23^2*s5^2 - 4*s1*s3*a11^2*a23^2 + 2*a11^2*a23*s5*s7 - 4*s1*a11^2*a23*s9 + a11^2*s7^2 - 4*s1*s10*a11^2 - 2*a11*a12*a21*a22*s4^2 + 8*s1*s2*a11*a12*a21*a22 - 2*a11*a12*a21*a23*s4*s5 + 4*s1*a11*a12*a21*a23*s6 - 2*a11*a12*a21*s4*s7 + 4*s1*a11*a12*a21*s8 - 2*a11*a12*a22*a23*s4*s6 + 4*s2*a11*a12*a22*a23*s5 - 2*a11*a12*a22*s4*s8 + 4*s2*a11*a12*a22*s7 - 4*s3*a11*a12*a23^2*s4 + 2*a11*a12*a23^2*s5*s6 - 4*a11*a12*a23*s4*s9 + 2*a11*a12*a23*s5*s8 + 2*a11*a12*a23*s6*s7 - 4*s10*a11*a12*s4 + 2*a11*a12*s7*s8 - 2*a11*a13*a21*a22*s4*s5 + 4*s1*a11*a13*a21*a22*s6 - 2*a11*a13*a21*a23*s5^2 + 8*s1*s3*a11*a13*a21*a23 - 2*a11*a13*a21*s5*s7 + 4*s1*a11*a13*a21*s9 + 2*a11*a13*a22^2*s4*s6 - 4*s2*a11*a13*a22^2*s5 + 4*s3*a11*a13*a22*a23*s4 - 2*a11*a13*a22*a23*s5*s6 + 2*a11*a13*a22*s4*s9 - 4*a11*a13*a22*s5*s8 + 2*a11*a13*a22*s6*s7 - 2*a11*a13*a23*s5*s9 + 4*s3*a11*a13*a23*s7 - 4*s10*a11*a13*s5 + 2*a11*a13*s7*s9 - 2*a11*a21*a22*s4*s7 + 4*s1*a11*a21*a22*s8 - 2*a11*a21*a23*s5*s7 + 4*s1*a11*a21*a23*s9 - 2*a11*a21*s7^2 + 8*s1*s10*a11*a21 + 2*a11*a22^2*s4*s8 - 4*s2*a11*a22^2*s7 + 2*a11*a22*a23*s4*s9 + 2*a11*a22*a23*s5*s8 - 4*a11*a22*a23*s6*s7 + 4*s10*a11*a22*s4 - 2*a11*a22*s7*s8 + 2*a11*a23^2*s5*s9 - 4*s3*a11*a23^2*s7 + 4*s10*a11*a23*s5 - 2*a11*a23*s7*s9 + a12^2*a21^2*s4^2 - 4*s1*s2*a12^2*a21^2 + 2*a12^2*a21*a23*s4*s6 - 4*s2*a12^2*a21*a23*s5 + 2*a12^2*a21*s4*s8 - 4*s2*a12^2*a21*s7 + a12^2*a23^2*s6^2 - 4*s2*s3*a12^2*a23^2 + 2*a12^2*a23*s6*s8 - 4*s2*a12^2*a23*s9 + a12^2*s8^2 - 4*s2*s10*a12^2 + 2*a12*a13*a21^2*s4*s5 - 4*s1*a12*a13*a21^2*s6 - 2*a12*a13*a21*a22*s4*s6 + 4*s2*a12*a13*a21*a22*s5 + 4*s3*a12*a13*a21*a23*s4 - 2*a12*a13*a21*a23*s5*s6 + 2*a12*a13*a21*s4*s9 + 2*a12*a13*a21*s5*s8 - 4*a12*a13*a21*s6*s7 - 2*a12*a13*a22*a23*s6^2 + 8*s2*s3*a12*a13*a22*a23 - 2*a12*a13*a22*s6*s8 + 4*s2*a12*a13*a22*s9 - 2*a12*a13*a23*s6*s9 + 4*s3*a12*a13*a23*s8 - 4*s10*a12*a13*s6 + 2*a12*a13*s8*s9 + 2*a12*a21^2*s4*s7 - 4*s1*a12*a21^2*s8 - 2*a12*a21*a22*s4*s8 + 4*s2*a12*a21*a22*s7 + 2*a12*a21*a23*s4*s9 - 4*a12*a21*a23*s5*s8 + 2*a12*a21*a23*s6*s7 + 4*s10*a12*a21*s4 - 2*a12*a21*s7*s8 - 2*a12*a22*a23*s6*s8 + 4*s2*a12*a22*a23*s9 - 2*a12*a22*s8^2 + 8*s2*s10*a12*a22 + 2*a12*a23^2*s6*s9 - 4*s3*a12*a23^2*s8 + 4*s10*a12*a23*s6 - 2*a12*a23*s8*s9 + a13^2*a21^2*s5^2 - 4*s1*s3*a13^2*a21^2 - 4*s3*a13^2*a21*a22*s4 + 2*a13^2*a21*a22*s5*s6 + 2*a13^2*a21*s5*s9 - 4*s3*a13^2*a21*s7 + a13^2*a22^2*s6^2 - 4*s2*s3*a13^2*a22^2 + 2*a13^2*a22*s6*s9 - 4*s3*a13^2*a22*s8 + a13^2*s9^2 - 4*s3*s10*a13^2 + 2*a13*a21^2*s5*s7 - 4*s1*a13*a21^2*s9 - 4*a13*a21*a22*s4*s9 + 2*a13*a21*a22*s5*s8 + 2*a13*a21*a22*s6*s7 - 2*a13*a21*a23*s5*s9 + 4*s3*a13*a21*a23*s7 + 4*s10*a13*a21*s5 - 2*a13*a21*s7*s9 + 2*a13*a22^2*s6*s8 - 4*s2*a13*a22^2*s9 - 2*a13*a22*a23*s6*s9 + 4*s3*a13*a22*a23*s8 + 4*s10*a13*a22*s6 - 2*a13*a22*s8*s9 - 2*a13*a23*s9^2 + 8*s3*s10*a13*a23 + a21^2*s7^2 - 4*s1*s10*a21^2 - 4*s10*a21*a22*s4 + 2*a21*a22*s7*s8 - 4*s10*a21*a23*s5 + 2*a21*a23*s7*s9 + a22^2*s8^2 - 4*s2*s10*a22^2 - 4*s10*a22*a23*s6 + 2*a22*a23*s8*s9 + a23^2*s9^2 - 4*s3*s10*a23^2;];
            
            u_value(1,:) = (-u_coeff(2) + sqrt(u_coeff(2)^2 - 4*u_coeff(1)*u_coeff(3)))/(2*u_coeff(1));
            u_value(2,:) = (-u_coeff(2) - sqrt(u_coeff(2)^2 - 4*u_coeff(1)*u_coeff(3)))/(2*u_coeff(1));
            u_value = unique(u_value);
            u_value = u_value(imag(u_value)==0);
            
            u_value(u_value <0) = [];u_value(u_value > 1) = [];
            if ~isempty(u_value)
                for i = 1:size(u_value,1)
                    v_coeff = v_coeff_u(u_value(i));
                    v_value = -v_coeff(2)/(2*v_coeff(1));
                    v_value = v_value(imag(v_value)==0);
                    v_value(v_value <0) = [];v_value(v_value > 1) = [];
                    %% find the corresponding pose by finding the same unit vector
                    if ~isempty(v_value)
                        for ii = 1:size(v_value,2)
                            %% check if out of surface boundary
                            tmp_val = uv_equ(u_value(i),v_value(ii));
                            if tmp_val(1) <= obj.surface_bound(2) && tmp_val(1) >= obj.surface_bound(1) && ...
                                    tmp_val(2) <= obj.surface_bound(4) && tmp_val(2) >= obj.surface_bound(3) &&...
                                    tmp_val(3) <= obj.surface_bound(6) && tmp_val(3) >= obj.surface_bound(5)
                                q_intersected = [q_intersected,(q_end - q_begin)*u_value(i) + q_begin];
                                intersected_pts = [intersected_pts, tmp_val];
                            end
                            
                        end
                    end
                    %
                end
            end
            
            
        end
        function [q_intersected,intersected_pts] = Edges2QuadSurfIntersection(obj,QuadSurfCoeff,AttPts,t_equ,uv_equ,base_att,q_begin,q_end)
            
            s1 = QuadSurfCoeff(1);
            s2 = QuadSurfCoeff(2);
            s3 = QuadSurfCoeff(3);
            s4 = QuadSurfCoeff(4);
            s5 = QuadSurfCoeff(5);
            s6 = QuadSurfCoeff(6);
            s7 = QuadSurfCoeff(7);
            s8 = QuadSurfCoeff(8);
            s9 = QuadSurfCoeff(9);
            s10 = QuadSurfCoeff(10);
            
            %             syms t u;
            q_intersected = [];intersected_pts = [];
            att_sequence = [3 2; 3 1; 2 1 ];
            
            for i = 1:3
                b1 = AttPts(att_sequence(i,1),1);
                b2 = AttPts(att_sequence(i,1),2);
                b3 = AttPts(att_sequence(i,1),3);
                
                c1 = AttPts(att_sequence(i,2),1);
                c2 = AttPts(att_sequence(i,2),2);
                c3 = AttPts(att_sequence(i,2),3);
                
                %                 para_equ = t_equ{i}(t);
                %                 f_1 =@(t) obj.QuadSurf(para_equ(1),para_equ(2),para_equ(3));
                %                 [t_numerator,t_denominator] = numden(f_1(t));
                %                 [t_coeff1,t_degree] = coeffs(t_numerator,t);
                
                t_coeff = [s1*b1^2 + s4*b1*b2 + s5*b1*b3 - 2*s1*b1*c1 - s4*b1*c2 - s5*b1*c3 + s2*b2^2 + s6*b2*b3 - s4*b2*c1 - 2*s2*b2*c2 - s6*b2*c3 + s3*b3^2 - s5*b3*c1 - s6*b3*c2 - 2*s3*b3*c3 + s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s2*c2^2 + s6*c2*c3 + s3*c3^2;
                    b1*s7 + b2*s8 + b3*s9 - c1*s7 - c2*s8 - c3*s9 - 2*c1^2*s1 - 2*c2^2*s2 - 2*c3^2*s3 + 2*b1*c1*s1 + 2*b2*c2*s2 + b1*c2*s4 + b2*c1*s4 + b1*c3*s5 + b3*c1*s5 + 2*b3*c3*s3 + b2*c3*s6 + b3*c2*s6 - 2*c1*c2*s4 - 2*c1*c3*s5 - 2*c2*c3*s6;
                    s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s7*c1 + s2*c2^2 + s6*c2*c3 + s8*c2 + s3*c3^2 + s9*c3 + s10];
                
                t_ans(1,:) = (-t_coeff(2) + sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                t_ans(2,:) = (-t_coeff(2) - sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                t_ans = unique(t_ans);
                t_ans = t_ans(imag(t_ans)==0);
                t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                if ~isempty(t_ans)
                    for ii = 1:size(t_ans,1)
                        %% check if out of surface boundary
                        %                         tmp_val = double(t_equ{i}(t_ans(ii)));
                        
                        tmp_val =  [c1 + t_ans(ii)*(b1 - c1);
                            c2 + t_ans(ii)*(b2 - c2);
                            c3 + t_ans(ii)*(b3 - c3)];
                        
                        if tmp_val(1) <= obj.surface_bound(2) && tmp_val(1) >= obj.surface_bound(1) && ...
                                tmp_val(2) <= obj.surface_bound(4) && tmp_val(2) >= obj.surface_bound(3) &&...
                                tmp_val(3) <= obj.surface_bound(6) && tmp_val(3) >= obj.surface_bound(5)
                            intersected_pts = [intersected_pts, tmp_val];
                            q_intersected = [q_intersected,(q_end - q_begin)*t_ans(ii) + q_begin];
                        end
                    end
                end
            end
        end
        
        %% function to check the intersected interval valid
        function has_intersected = IntervalVerify(obj,model,QuadSurfCoeff,q1,q2)
            %             syms t;
            [Att1,Att2] = obj.GetSegmentData(model,(q1+q2)/2);
            
            s1 = QuadSurfCoeff(1);
            s2 = QuadSurfCoeff(2);
            s3 = QuadSurfCoeff(3);
            s4 = QuadSurfCoeff(4);
            s5 = QuadSurfCoeff(5);
            s6 = QuadSurfCoeff(6);
            s7 = QuadSurfCoeff(7);
            s8 = QuadSurfCoeff(8);
            s9 = QuadSurfCoeff(9);
            s10 = QuadSurfCoeff(10);
            
            for i = 1:obj.numCables
                b1 = Att2(i,1);
                b2 = Att2(i,2);
                b3 = Att2(i,3);
                
                c1 = Att1(i,1);
                c2 = Att1(i,2);
                c3 = Att1(i,3);
                %                 Check_seg_t =   (Att2(i,:) - Att1(i,:))'.*t + Att1(i,:)';
                %                 f_1 =@(t) obj.QuadSurf(Check_seg_t(1),Check_seg_t(2),Check_seg_t(3));
                %                 [t_coeff,~] = coeffs(f_1(t),t);
                
                t_coeff = [s1*b1^2 + s4*b1*b2 + s5*b1*b3 - 2*s1*b1*c1 - s4*b1*c2 - s5*b1*c3 + s2*b2^2 + s6*b2*b3 - s4*b2*c1 - 2*s2*b2*c2 - s6*b2*c3 + s3*b3^2 - s5*b3*c1 - s6*b3*c2 - 2*s3*b3*c3 + s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s2*c2^2 + s6*c2*c3 + s3*c3^2;
                    b1*s7 + b2*s8 + b3*s9 - c1*s7 - c2*s8 - c3*s9 - 2*c1^2*s1 - 2*c2^2*s2 - 2*c3^2*s3 + 2*b1*c1*s1 + 2*b2*c2*s2 + b1*c2*s4 + b2*c1*s4 + b1*c3*s5 + b3*c1*s5 + 2*b3*c3*s3 + b2*c3*s6 + b3*c2*s6 - 2*c1*c2*s4 - 2*c1*c3*s5 - 2*c2*c3*s6;
                    s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s7*c1 + s2*c2^2 + s6*c2*c3 + s8*c2 + s3*c3^2 + s9*c3 + s10];
                
                t_condition = round(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3),5);
%                 t_ans(1,:) = (-t_coeff(2) + sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
%                 t_ans(2,:) = (-t_coeff(2) - sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
%                 t_ans = t_ans(imag(t_ans)==0);
%                 t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                
                if t_condition >= 0
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
        
        %% function to get the coeffcient of the quadratic surface
        function Q  = GetQuadSurfCoeff(obj)
            syms x y z;
            Q = zeros(1,10);
            [coeff_f,var_f] = coeffs(obj.QuadSurf(x,y,z));
            for i = 1:size(coeff_f,2)
                if isequal(var_f(i),x^2)
                    Q(1) = coeff_f(i);
                elseif isequal(var_f(i),y^2)
                    Q(2) = coeff_f(i);
                elseif isequal(var_f(i),z^2)
                    Q(3) = coeff_f(i);
                elseif isequal(var_f(i),x*y)
                    Q(4) = coeff_f(i);
                elseif isequal(var_f(i),x*z)
                    Q(5) = coeff_f(i);
                elseif isequal(var_f(i),y*z)
                    Q(6) = coeff_f(i);
                elseif isequal(var_f(i),x)
                    Q(7) = coeff_f(i);
                elseif isequal(var_f(i),y)
                    Q(8) = coeff_f(i);
                elseif isequal(var_f(i),z)
                    Q(9) = coeff_f(i);
                else
                    Q(10) = coeff_f(i);
                end
                
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

