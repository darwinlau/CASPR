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
        type = WorkspaceRayConditionType.INTERFERENCE_CABLE_CABLE;
        
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
            % Variable initialisation
            syms u v t;
            q_zero = zeros(obj.numDofs, 1);
            free_variable_index = ws_ray.free_variable_index;
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            
            if is_dof_translation
                q_begin = [ws_ray.free_variable_range(1);ws_ray.fixed_variables];
                q_end = [ws_ray.free_variable_range(2);ws_ray.fixed_variables];
                % Att_pts{1} -> base point,  Att_pts{2}-> start point,  Att_pts{3}-> end point
                [Att_pts{1},Att_pts{2}] = obj.GetSegmentData(model,q_begin);
                [~,Att_pts{3}] = obj.GetSegmentData(model,q_end);
                
                ob1 = fimplicit3(obj.QuadSurf,[-2 2 -2 2 0 2.5],'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.4)
                hold on;
                
                for i = 1:obj.numCables
                    h=patch('Faces',1:3,'Vertices',[Att_pts{3}(i,:);Att_pts{2}(i,:);Att_pts{1}(i,:)]);
                    set(h,'FaceColor','r','EdgeColor','k','LineWidth',2,'FaceAlpha',0.5);
                    %% parametric form f(u,v) of the cable segment surface
                    parametric_cable_surf = (((Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*u +  Att_pts{2}(i,:)') - Att_pts{1}(i,:)').*v + Att_pts{1}(i,:)';
                    parametric_cable_surf_uv = @(u,v) (((Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*u +  Att_pts{2}(i,:)') - Att_pts{1}(i,:)').*v + Att_pts{1}(i,:)';
                    q_intersected = [];
                    %% boundary curves intersection
                    intersected_pts = [];
                    Segment{1} = (Att_pts{3}(i,:) - Att_pts{2}(i,:))'.*t + Att_pts{2}(i,:)';
                    Segment{2} = (Att_pts{3}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    Segment{3} = (Att_pts{2}(i,:) - Att_pts{1}(i,:))'.*t + Att_pts{1}(i,:)';
                    for ii = 1:3
                        f_1 =@(t) obj.QuadSurf(Segment{ii}(1),Segment{ii}(2),Segment{ii}(3));
                        t_ans = double(solve(f_1(t)));
                        t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                        for iii = 1:size(t_ans,1)
                        intersected_pts = [intersected_pts,double(subs(Segment{ii},t_ans(iii)))];
                        end
                        for iii = 1:size(intersected_pts,2)
%                             angle(iii) = acos(((Att_pts{2}(i,:) - Att_pts{1}(i,:)) * (intersected_pts(:,iii)' - Att_pts{1}(i,:))')/...
%                                 (norm((Att_pts{2}(i,:)' - Att_pts{1}(i,:))) * norm((intersected_pts(:,iii) - Att_pts{1}(i,:)))));
                            %% finding the corresponding poses of intersection                            
                            unit_vec_1 = parametric_cable_surf_uv(u,1) - Att_pts{1}(i,:)';
                            unit_vec_2 = intersected_pts(:,iii) - Att_pts{1}(i,:)';
                            u_value = double(solve((unit_vec_1(1)^2 +  unit_vec_1(2)^2 +  unit_vec_1(3)^2)*unit_vec_2(1)^2/norm(unit_vec_2)^2 - unit_vec_1(1)^2));
                            u_value(u_value <0) = [];u_value(u_value > 1) = [];
                            q_intersected = [q_intersected,(q_end - q_begin)*u_value + q_begin]
                        end
                    end
                    
                    i
                    
                    %                     f_1 =@(u,v) obj.QuadSurf(parametric_cable_surf(1),parametric_cable_surf(2),parametric_cable_surf(3));
                    %                     [v_coeff,v_degree] = coeffs(f_1(u,v),v);
                    %
                    %                     f_2 = v_coeff(2)*v_coeff(2) - 4*v_coeff(3)*v_coeff(1);
                    %                     [u_numerator,u_denominator] = numden(f_2);
                    %                     [u_coeff,u_degree] = coeffs(u_numerator,u);
                    %                     u_ans = unique(double((roots(u_coeff))));
                    %                     u_ans(u_ans <0) = [];u_ans(u_ans > 1) = [];
                    %
                    %                     for ii = 1:max(size(u_ans))
                    %                         parametric_cable_surf_uv(u_ans(1),v)
                    %                         v_coeff = subs(v_coeff,u_ans(1));
                    %                         v_ans = double(roots(v_coeff));
                    %                         v_ans(v_ans <0) = [];v_ans(v_ans > 1) = [];
                    %
                    %                     end
                end
                
            else
                
            end
            
        end
        
    end
    
    methods (Access = private)
        function [base_att_pt,EE_att_pt] = GetSegmentData(~,model,q)
            
            model.update(q,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1))
            
            for i = 1:model.numCables
                base_att_pt(i,:) = model.cableModel.cables{1,i}.attachments{1,1}.r_OA';
                EE_att_pt(i,:) = model.cableModel.cables{1,i}.attachments{1,2}.r_OA';
            end
        end
    end
end

