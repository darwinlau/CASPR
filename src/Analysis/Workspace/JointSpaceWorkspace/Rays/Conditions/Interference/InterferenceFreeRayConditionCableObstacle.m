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
            
            for surf_ind = 1:size(obj.QuadSurf,1)
                QuadSurfCoeff  = obj.GetQuadSurfCoeff(obj.QuadSurf{surf_ind});
                %             tic
                if is_dof_translation
                    
                    for i = 1:obj.numCables
                        % three attachements from base frame
                        AttPts = [Cable_Surface_End_pts{1}(i,:);Cable_Surface_End_pts{2}(i,:);Cable_Surface_End_pts{3}(i,:)];
                        % parametric form f(u,v) of the cable segment surface
                        parametric_cable_surf_uv = @(u,v) (((Cable_Surface_End_pts{3}(i,:) - Cable_Surface_End_pts{2}(i,:))'.*u +  Cable_Surface_End_pts{2}(i,:)') - Cable_Surface_End_pts{1}(i,:)').*v + Cable_Surface_End_pts{1}(i,:)';
                        % find the intersection of the triangle surface to the quadratic surface
                        [q_intersected_at_surf,intersected_pts_at_surf] = obj.ParametericSurfaceIntersectionTranslation(QuadSurfCoeff,AttPts,parametric_cable_surf_uv,q_begin,q_end,surf_ind);
                        all_intersection_poses = [all_intersection_poses, q_intersected_at_surf];
                        all_intersected_pts = [all_intersected_pts,intersected_pts_at_surf];
                        
                        %% Three edges of the triangle surface
                        % find the intersection of three edges with the
                        % quadratic surface
                        [q_intersected_at_edges,intersected_pts_at_edges] = Edges2QuadSurfIntersection(obj,QuadSurfCoeff,AttPts,q_begin,q_end,surf_ind);
                        
                        all_intersection_poses = [all_intersection_poses, q_intersected_at_edges];
                        all_intersected_pts = [all_intersected_pts,intersected_pts_at_edges];
                        
                    end
                    
                else
                    valid_range = [tan(q_begin(free_variable_index)/2), tan(q_end(free_variable_index)/2)];
                    Si = GetSegmentEquation(obj,model,q_begin,q_end);
                    
                    for i = 1:obj.numCables
                        rOAi = model.cableModel.cables{1,i}.attachments{1,1}.r_OA;
                        rGAi = model.cableModel.cables{1,i}.attachments{1,2}.r_GA;
                        %                     rOAi = [a1;a2;a3];
                        AttPts = [Cable_Surface_End_pts{1}(i,:);Cable_Surface_End_pts{2}(i,:);Cable_Surface_End_pts{3}(i,:)];
                        Si_u =@(u) Si{i}(u);
                        parametric_cable_surf_uv =@(u,v) Si_u(u)* v + rOAi;
                        [q_intersected,intersected_pts] = ParametericSurfaceIntersectionOrientation(obj,QuadSurfCoeff,rOAi,rGAi,parametric_cable_surf_uv,...
                            q_begin,q_end,valid_range,surf_ind);
                        
                        all_intersection_poses = [all_intersection_poses, q_intersected];
                        all_intersected_pts = [all_intersected_pts,intersected_pts];
                        
                        %% boundary curves intersection
                        Segment{1} =@(t) Si_u(t) + rOAi;
                        Segment{2} =@(t) (Cable_Surface_End_pts{3}(i,:) - Cable_Surface_End_pts{1}(i,:)).'.*t + Cable_Surface_End_pts{1}(i,:).';
                        Segment{3} =@(t) (Cable_Surface_End_pts{2}(i,:) - Cable_Surface_End_pts{1}(i,:)).'.*t + Cable_Surface_End_pts{1}(i,:).';
                        [q_intersected,intersected_pts] = Curve2QuadSurfIntersection(obj,QuadSurfCoeff,rGAi,AttPts,Segment,q_begin,q_end,valid_range,surf_ind);
                        
                        all_intersection_poses = [all_intersection_poses, q_intersected];
                        all_intersected_pts = [all_intersected_pts,intersected_pts];
                        
                    end
                end
                
                %% verify the intersection interval
                all_intersection_poses = unique(round(all_intersection_poses',obj.ROUNDING_DIGIT),'rows')';
            end
            pre_has_intersected = 1;
            for i = 1:size(all_intersection_poses,2) - 1
                for surf_ind =1:size(obj.QuadSurf,1)
                    QuadSurfCoeff = obj.GetQuadSurfCoeff(obj.QuadSurf{surf_ind});
                    has_intersected = obj.IntervalVerify(model,QuadSurfCoeff,all_intersection_poses(:,i),all_intersection_poses(:,i+1));
                    if has_intersected == 0
                        if pre_has_intersected ~= 0
                            intervals(intervals_count,:) = [all_intersection_poses(free_variable_index,i),all_intersection_poses(free_variable_index,i+1)];
                            intervals_count = intervals_count + 1;
                        else
                            intervals(intervals_count-1 ,end) = all_intersection_poses(free_variable_index,i+1);
                        end
                    end
                end
                pre_has_intersected = has_intersected;
            end
            %             toc
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
        
        %% function to get the cable segment equation
        function [Si] = GetSegmentEquation(obj,model,q_begin,q_end)
            % get start and end angles
            %             model.update(q_end,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
            %             R1 = model.bodyModel.R_0ks;
            %             R1_index = find(R1==1);
            %             model.update(q_begin,zeros(model.numDofs,1), zeros(model.numDofs,1),zeros(model.numDofs,1));
            %             R2 = model.bodyModel.R_0ks;
            %             R2_index = find(R2==1);
            %            [Rotational_axis_ind,~]=intersect(R1_index,R2_index);
            Rotational_axis_ind =  find(~(q_begin == q_end));
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
            for i = 1:model.numCables
                r_OA_i = model.cableModel.cables{1,i}.attachments{1,1}.r_OA;
                r_GA_i = model.cableModel.cables{1,i}.attachments{1,2}.r_GA;
                
                Si{i} =@(u) (q_begin(obj.areDofsTranslation) - r_OA_i) + Rot_Mat(u)*r_GA_i;
            end
        end
        
        %% function to calculate the intersected poses between quad-surface and cable segment bounded surface
        function [q_intersected,intersected_pts] = ParametericSurfaceIntersectionOrientation(obj,QuadSurfCoeff,r_OA_i,r_GA_i,uv_equ,q_begin,q_end,u_range,surf_ind)
            intersected_pts = [];q_intersected = [];
            %%
            a1 = r_OA_i(1);
            a2 = r_OA_i(2);
            a3 = r_OA_i(3);
            b1 = r_GA_i(1);
            b2 = r_GA_i(2);
            b3 = r_GA_i(3);
            
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
            %%
             %%%%%%% A*v^2 + B*v + C = 0, coefficient of v is shown below in
            %%%%%%% the form of v_coeff = [A;B;C;...]
            v_coeff_u =@(u) [s1*(a1 + (b1*(u^2 - 1))/(u^2 + 1) - (2*b3*u)/(u^2 + 1) - 1/10)^2 + s3*(a3 + (b3*(u^2 - 1))/(u^2 + 1) + (2*b1*u)/(u^2 + 1) - 11/20)^2 + s2*(b2 - a2 + 1/5)^2 - s4*(b2 - a2 + 1/5)*(a1 + (b1*(u^2 - 1))/(u^2 + 1) - (2*b3*u)/(u^2 + 1) - 1/10) - s6*(b2 - a2 + 1/5)*(a3 + (b3*(u^2 - 1))/(u^2 + 1) + (2*b1*u)/(u^2 + 1) - 11/20) + s5*(a1 + (b1*(u^2 - 1))/(u^2 + 1) - (2*b3*u)/(u^2 + 1) - 1/10)*(a3 + (b3*(u^2 - 1))/(u^2 + 1) + (2*b1*u)/(u^2 + 1) - 11/20);
                s8*(b2 - a2 + 1/5) - s7*(a1 + (b1*(u^2 - 1))/(u^2 + 1) - (2*b3*u)/(u^2 + 1) - 1/10) - s9*(a3 + (b3*(u^2 - 1))/(u^2 + 1) + (2*b1*u)/(u^2 + 1) - 11/20) - 2*a1*s1*(a1 + (b1*(u^2 - 1))/(u^2 + 1) - (2*b3*u)/(u^2 + 1) - 1/10) - a2*s4*(a1 + (b1*(u^2 - 1))/(u^2 + 1) - (2*b3*u)/(u^2 + 1) - 1/10) - a3*s5*(a1 + (b1*(u^2 - 1))/(u^2 + 1) - (2*b3*u)/(u^2 + 1) - 1/10) - a1*s5*(a3 + (b3*(u^2 - 1))/(u^2 + 1) + (2*b1*u)/(u^2 + 1) - 11/20) - 2*a3*s3*(a3 + (b3*(u^2 - 1))/(u^2 + 1) + (2*b1*u)/(u^2 + 1) - 11/20) - a2*s6*(a3 + (b3*(u^2 - 1))/(u^2 + 1) + (2*b1*u)/(u^2 + 1) - 11/20) + 2*a2*s2*(b2 - a2 + 1/5) + a1*s4*(b2 - a2 + 1/5) + a3*s6*(b2 - a2 + 1/5);
                s1*a1^2 + s4*a1*a2 + s5*a1*a3 + s7*a1 + s2*a2^2 + s6*a2*a3 + s8*a2 + s3*a3^2 + s9*a3 + s10];
            
            u_coeff = [16*s7*s8 - 64*s2*s10 - 484*s3*s10 - 32*s4*s10 - 88*s5*s10 - 16*s1*s10 - 176*s6*s10 + 44*s7*s9 + 88*s8*s9 - 80*a1*s7^2 - 160*a2*s8^2 - 440*a3*s9^2 - 80*b1*s7^2 + 160*b2*s8^2 - 440*b3*s9^2 + 4*s7^2 + 16*s8^2 + 121*s9^2 + 16*a1^2*s4^2 + 121*a1^2*s5^2 + 4*a2^2*s4^2 + 400*a1^2*s7^2 + 121*a2^2*s6^2 + 4*a3^2*s5^2 + 16*a3^2*s6^2 + 400*a2^2*s8^2 + 400*a3^2*s9^2 + 400*b1^2*s7^2 + 400*b2^2*s8^2 + 400*b3^2*s9^2 - 64*a1^2*s1*s2 - 484*a1^2*s1*s3 - 16*a2^2*s1*s2 - 484*a2^2*s2*s3 - 16*a3^2*s1*s3 - 176*a1^2*s1*s6 - 64*a3^2*s2*s3 - 88*a2^2*s2*s5 - 320*a1^2*s1*s8 + 88*a1^2*s4*s5 - 32*a3^2*s3*s4 - 880*a1^2*s1*s9 - 160*a2^2*s2*s7 - 1600*a1^2*s1*s10 + 160*a1^2*s4*s7 + 44*a2^2*s4*s6 + 440*a1^2*s5*s7 - 880*a2^2*s2*s9 - 160*a3^2*s3*s7 - 1600*a2^2*s2*s10 + 80*a2^2*s4*s8 - 320*a3^2*s3*s8 + 16*a3^2*s5*s6 + 440*a2^2*s6*s8 - 1600*a3^2*s3*s10 + 80*a3^2*s5*s9 + 160*a3^2*s6*s9 - 1600*b1^2*s1*s10 - 1600*b2^2*s2*s10 - 1600*b3^2*s3*s10 + 160*a1^2*b2*s4^2 - 80*a2^2*b1*s4^2 - 440*a1^2*b3*s5^2 - 80*a3^2*b1*s5^2 - 440*a2^2*b3*s6^2 + 160*a3^2*b2*s6^2 + 32*a1*s1*s8 - 64*a1*s2*s7 + 88*a1*s1*s9 - 484*a1*s3*s7 - 16*a2*s1*s8 + 32*a2*s2*s7 + 320*a1*s1*s10 - 16*a1*s4*s7 + 32*a1*s4*s8 - 44*a1*s5*s7 + 176*a2*s2*s9 - 484*a2*s3*s8 + 8*a2*s4*s7 - 16*a3*s1*s9 + 88*a3*s3*s7 + 88*a1*s4*s9 + 88*a1*s5*s8 - 176*a1*s6*s7 + 640*a2*s2*s10 - 16*a2*s4*s8 - 64*a3*s2*s9 + 176*a3*s3*s8 + 320*a1*s4*s10 + 242*a1*s5*s9 + 44*a2*s4*s9 - 88*a2*s5*s8 + 44*a2*s6*s7 + 8*a3*s5*s7 + 880*a1*s5*s10 - 160*a1*s7*s8 + 160*a2*s4*s10 - 88*a2*s6*s8 + 1760*a3*s3*s10 - 32*a3*s4*s9 + 16*a3*s5*s8 + 16*a3*s6*s7 - 440*a1*s7*s9 + 242*a2*s6*s9 - 80*a2*s7*s8 - 44*a3*s5*s9 + 32*a3*s6*s8 + 880*a2*s6*s10 + 160*a3*s5*s10 - 88*a3*s6*s9 - 440*a2*s8*s9 + 320*a3*s6*s10 - 80*a3*s7*s9 - 160*a3*s8*s9 + 320*b1*s1*s10 - 640*b2*s2*s10 + 320*b1*s4*s10 + 880*b1*s5*s10 - 160*b1*s7*s8 - 160*b2*s4*s10 + 1760*b3*s3*s10 - 440*b1*s7*s9 + 80*b2*s7*s8 - 880*b2*s6*s10 + 160*b3*s5*s10 + 440*b2*s8*s9 + 320*b3*s6*s10 - 80*b3*s7*s9 - 160*b3*s8*s9 + 400*a1^2*b2^2*s4^2 + 400*a2^2*b1^2*s4^2 + 400*a1^2*b3^2*s5^2 + 400*a3^2*b1^2*s5^2 + 400*a2^2*b3^2*s6^2 + 400*a3^2*b2^2*s6^2 - 16*a1*a2*s4^2 - 44*a1*a3*s5^2 - 88*a2*a3*s6^2 + 800*a1*b1*s7^2 - 800*a2*b2*s8^2 + 800*a3*b3*s9^2 + 64*a1*a2*s1*s2 + 176*a1*a3*s1*s3 + 88*a1*a2*s1*s6 + 176*a1*a2*s2*s5 - 484*a1*a2*s3*s4 + 352*a2*a3*s2*s3 + 32*a1*a3*s1*s6 - 64*a1*a3*s2*s5 + 176*a1*a3*s3*s4 + 160*a1*a2*s1*s8 + 320*a1*a2*s2*s7 - 44*a1*a2*s4*s5 - 16*a2*a3*s1*s6 + 32*a2*a3*s2*s5 + 88*a2*a3*s3*s4 - 88*a1*a2*s4*s6 - 16*a1*a3*s4*s5 - 80*a1*a2*s4*s7 + 242*a1*a2*s5*s6 + 160*a1*a3*s1*s9 + 880*a1*a3*s3*s7 + 32*a1*a3*s4*s6 + 8*a2*a3*s4*s5 - 160*a1*a2*s4*s8 - 88*a1*a3*s5*s6 - 16*a2*a3*s4*s6 - 880*a1*a2*s4*s9 + 440*a1*a2*s5*s8 + 440*a1*a2*s6*s7 - 80*a1*a3*s5*s7 + 320*a2*a3*s2*s9 + 880*a2*a3*s3*s8 - 44*a2*a3*s5*s6 - 1600*a1*a2*s4*s10 + 160*a1*a3*s4*s9 - 320*a1*a3*s5*s8 + 160*a1*a3*s6*s7 + 800*a1*a2*s7*s8 - 440*a1*a3*s5*s9 + 80*a2*a3*s4*s9 + 80*a2*a3*s5*s8 - 160*a2*a3*s6*s7 - 1600*a1*a3*s5*s10 - 160*a2*a3*s6*s8 + 800*a1*a3*s7*s9 - 440*a2*a3*s6*s9 - 1600*a2*a3*s6*s10 + 800*a2*a3*s8*s9 - 320*a1*b1*s1*s8 - 880*a1*b1*s1*s9 + 160*a1*b2*s1*s8 - 640*a1*b2*s2*s7 + 320*a2*b1*s1*s8 - 320*a2*b1*s2*s7 - 3200*a1*b1*s1*s10 + 160*a1*b1*s4*s7 + 160*a2*b2*s2*s7 + 440*a1*b1*s5*s7 - 80*a1*b2*s4*s7 - 160*a1*b3*s1*s9 + 1760*a1*b3*s3*s7 - 160*a2*b1*s4*s7 + 320*a3*b1*s1*s9 - 880*a3*b1*s3*s7 + 320*a1*b2*s4*s8 + 160*a2*b1*s4*s8 + 880*a2*b2*s2*s9 + 440*a1*b2*s4*s9 + 440*a1*b2*s5*s8 - 880*a1*b2*s6*s7 + 80*a1*b3*s5*s7 - 440*a2*b1*s4*s9 + 880*a2*b1*s5*s8 - 440*a2*b1*s6*s7 + 3200*a2*b2*s2*s10 - 80*a2*b2*s4*s8 - 320*a2*b3*s2*s9 + 1760*a2*b3*s3*s8 - 160*a3*b1*s5*s7 - 640*a3*b2*s2*s9 + 880*a3*b2*s3*s8 - 160*a3*b3*s3*s7 + 1600*a1*b2*s4*s10 - 160*a1*b3*s4*s9 - 160*a1*b3*s5*s8 + 320*a1*b3*s6*s7 - 1600*a2*b1*s4*s10 + 320*a3*b1*s4*s9 - 160*a3*b1*s5*s8 - 160*a3*b1*s6*s7 - 320*a3*b3*s3*s8 - 800*a1*b2*s7*s8 - 880*a1*b3*s5*s9 + 800*a2*b1*s7*s8 - 440*a2*b2*s6*s8 - 80*a2*b3*s4*s9 + 160*a2*b3*s5*s8 - 80*a2*b3*s6*s7 + 440*a3*b1*s5*s9 - 160*a3*b2*s4*s9 + 80*a3*b2*s5*s8 + 80*a3*b2*s6*s7 - 1600*a1*b3*s5*s10 + 160*a2*b3*s6*s8 - 1600*a3*b1*s5*s10 + 320*a3*b2*s6*s8 - 3200*a3*b3*s3*s10 + 800*a1*b3*s7*s9 - 880*a2*b3*s6*s9 + 800*a3*b1*s7*s9 - 440*a3*b2*s6*s9 + 80*a3*b3*s5*s9 - 1600*a2*b3*s6*s10 + 1600*a3*b2*s6*s10 + 160*a3*b3*s6*s9 + 800*a2*b3*s8*s9 - 800*a3*b2*s8*s9 + 1600*b1*b2*s4*s10 - 800*b1*b2*s7*s8 - 1600*b1*b3*s5*s10 + 800*b1*b3*s7*s9 + 1600*b2*b3*s6*s10 - 800*b2*b3*s8*s9 + 160*a1*a2*b1*s4^2 - 80*a1*a2*b2*s4^2 + 440*a1*a3*b1*s5^2 + 80*a1*a3*b3*s5^2 - 440*a2*a3*b2*s6^2 + 160*a2*a3*b3*s6^2 - 640*a1^2*b2*s1*s2 + 320*a2^2*b1*s1*s2 + 1760*a1^2*b3*s1*s3 + 320*a3^2*b1*s1*s3 - 880*a1^2*b2*s1*s6 + 880*a2^2*b1*s2*s5 + 1760*a2^2*b3*s2*s3 - 640*a3^2*b2*s2*s3 + 320*a1^2*b3*s1*s6 + 320*a3^2*b1*s3*s4 - 1600*a1*b2^2*s2*s7 - 1600*a2*b1^2*s1*s8 - 1600*a1^2*b2*s1*s8 + 440*a1^2*b2*s4*s5 + 1600*a2^2*b1*s2*s7 + 160*a2^2*b3*s2*s5 - 160*a3^2*b2*s3*s4 - 160*a1^2*b3*s4*s5 - 440*a2^2*b1*s4*s6 - 1600*a1*b3^2*s3*s7 + 800*a2*b1^2*s4*s7 - 1600*a3*b1^2*s1*s9 + 800*a1^2*b2*s4*s7 + 1600*a1^2*b3*s1*s9 + 1600*a3^2*b1*s3*s7 + 800*a1*b2^2*s4*s8 - 800*a2^2*b1*s4*s8 - 80*a2^2*b3*s4*s6 - 160*a3^2*b1*s5*s6 - 1600*a2*b3^2*s3*s8 + 800*a3*b1^2*s5*s7 - 1600*a3*b2^2*s2*s9 - 800*a1^2*b3*s5*s7 + 1600*a2^2*b3*s2*s9 - 1600*a3^2*b2*s3*s8 + 80*a3^2*b2*s5*s6 + 800*a1*b3^2*s5*s9 - 800*a3^2*b1*s5*s9 + 800*a3*b2^2*s6*s8 - 800*a2^2*b3*s6*s8 + 800*a2*b3^2*s6*s9 + 800*a3^2*b2*s6*s9 - 1600*a1^2*b2^2*s1*s2 - 1600*a2^2*b1^2*s1*s2 - 1600*a1^2*b3^2*s1*s3 - 1600*a3^2*b1^2*s1*s3 - 1600*a2^2*b3^2*s2*s3 - 1600*a3^2*b2^2*s2*s3 - 640*a1*a2*b1*s1*s2 + 320*a1*a2*b2*s1*s2 - 1760*a1*a3*b1*s1*s3 - 880*a1*a2*b1*s1*s6 - 320*a1*a3*b3*s1*s3 + 880*a1*a2*b2*s2*s5 - 320*a1*a3*b1*s1*s6 + 1760*a2*a3*b2*s2*s3 - 1600*a1*a2*b1*s1*s8 + 440*a1*a2*b1*s4*s5 - 160*a1*a2*b3*s1*s6 - 320*a1*a2*b3*s2*s5 + 1760*a1*a2*b3*s3*s4 + 160*a1*a3*b2*s1*s6 - 640*a1*a3*b2*s2*s5 + 880*a1*a3*b2*s3*s4 + 320*a2*a3*b1*s1*s6 - 320*a2*a3*b1*s2*s5 - 880*a2*a3*b1*s3*s4 - 640*a2*a3*b3*s2*s3 + 1600*a1*a2*b2*s2*s7 + 160*a1*a3*b1*s4*s5 - 320*a1*a3*b3*s3*s4 + 160*a2*a3*b2*s2*s5 + 800*a1*a2*b1*s4*s7 - 440*a1*a2*b2*s4*s6 + 80*a1*a2*b3*s4*s5 - 1600*a1*a3*b1*s1*s9 - 80*a1*a3*b2*s4*s5 - 160*a2*a3*b1*s4*s5 - 160*a2*a3*b3*s3*s4 + 160*a1*a2*b3*s4*s6 + 320*a1*a3*b2*s4*s6 + 160*a2*a3*b1*s4*s6 - 800*a1*a2*b2*s4*s8 - 880*a1*a2*b3*s5*s6 + 800*a1*a3*b1*s5*s7 - 440*a1*a3*b2*s5*s6 - 1600*a1*a3*b3*s3*s7 + 440*a2*a3*b1*s5*s6 - 80*a2*a3*b2*s4*s6 + 160*a1*a3*b3*s5*s6 + 1600*a2*a3*b2*s2*s9 + 1600*a1*a2*b3*s4*s9 - 800*a1*a2*b3*s5*s8 - 800*a1*a2*b3*s6*s7 + 800*a1*a3*b2*s4*s9 - 1600*a1*a3*b2*s5*s8 + 800*a1*a3*b2*s6*s7 - 800*a2*a3*b1*s4*s9 - 800*a2*a3*b1*s5*s8 + 1600*a2*a3*b1*s6*s7 - 1600*a2*a3*b3*s3*s8 + 80*a2*a3*b3*s5*s6 + 800*a1*a3*b3*s5*s9 - 800*a2*a3*b2*s6*s8 + 800*a2*a3*b3*s6*s9 - 1600*a1*b1*b2*s1*s8 - 1600*a2*b1*b2*s2*s7 + 800*a1*b1*b2*s4*s7 + 1600*a1*b1*b3*s1*s9 - 800*a1*b1*b3*s5*s7 + 800*a2*b1*b2*s4*s8 + 1600*a3*b1*b3*s3*s7 - 1600*a2*b2*b3*s2*s9 - 800*a1*b2*b3*s4*s9 - 800*a1*b2*b3*s5*s8 + 1600*a1*b2*b3*s6*s7 + 800*a2*b1*b3*s4*s9 - 1600*a2*b1*b3*s5*s8 + 800*a2*b1*b3*s6*s7 + 1600*a3*b1*b2*s4*s9 - 800*a3*b1*b2*s5*s8 - 800*a3*b1*b2*s6*s7 - 1600*a3*b2*b3*s3*s8 + 800*a2*b2*b3*s6*s8 - 800*a3*b1*b3*s5*s9 + 800*a3*b2*b3*s6*s9 + 800*a1*a2*b1*b2*s4^2 - 800*a1*a3*b1*b3*s5^2 + 800*a2*a3*b2*b3*s6^2 - 1600*a1*a2*b3^2*s3*s4 - 1600*a1*a3*b2^2*s2*s5 - 1600*a2*a3*b1^2*s1*s6 + 800*a2*a3*b1^2*s4*s5 + 800*a1*a3*b2^2*s4*s6 + 800*a1*a2*b3^2*s5*s6 + 1600*a1^2*b2*b3*s1*s6 - 1600*a2^2*b1*b3*s2*s5 + 1600*a3^2*b1*b2*s3*s4 - 800*a1^2*b2*b3*s4*s5 + 800*a2^2*b1*b3*s4*s6 - 800*a3^2*b1*b2*s5*s6 - 3200*a1*a2*b1*b2*s1*s2 + 3200*a1*a3*b1*b3*s1*s3 + 1600*a1*a2*b1*b3*s1*s6 - 1600*a1*a3*b1*b2*s1*s6 - 1600*a1*a2*b2*b3*s2*s5 - 1600*a2*a3*b1*b2*s2*s5 - 3200*a2*a3*b2*b3*s2*s3 - 800*a1*a2*b1*b3*s4*s5 + 800*a1*a3*b1*b2*s4*s5 - 1600*a1*a3*b2*b3*s3*s4 + 1600*a2*a3*b1*b3*s3*s4 + 800*a1*a2*b2*b3*s4*s6 + 800*a2*a3*b1*b2*s4*s6 + 800*a1*a3*b2*b3*s5*s6 - 800*a2*a3*b1*b3*s5*s6;
                160*b3*s7^2 - 880*b1*s9^2 - 3200*b1^2*s5*s10 + 1600*b1^2*s7*s9 + 3200*b3^2*s5*s10 - 1600*b3^2*s7*s9 - 880*a1^2*b1*s5^2 - 880*a2^2*b1*s6^2 + 160*a2^2*b3*s4^2 + 160*a3^2*b3*s5^2 + 3520*b1*s3*s10 - 640*b3*s1*s10 + 320*b1*s5*s10 + 640*b1*s6*s10 - 160*b1*s7*s9 - 640*b3*s4*s10 - 320*b1*s8*s9 - 1760*b3*s5*s10 + 320*b3*s7*s8 + 880*b3*s7*s9 - 1600*a1*b3*s7^2 + 1600*a3*b1*s9^2 - 1600*b1*b3*s7^2 + 1600*b1*b3*s9^2 - 320*a1*b1*s1*s9 + 3520*a1*b1*s3*s7 + 640*a1*b3*s1*s8 + 160*a1*b1*s5*s7 + 1760*a1*b3*s1*s9 - 640*a2*b1*s2*s9 + 3520*a2*b1*s3*s8 - 640*a2*b3*s1*s8 + 640*a2*b3*s2*s7 - 320*a3*b1*s3*s7 - 320*a1*b1*s4*s9 - 320*a1*b1*s5*s8 + 640*a1*b1*s6*s7 + 6400*a1*b3*s1*s10 - 320*a1*b3*s4*s7 - 640*a3*b1*s3*s8 - 1760*a1*b1*s5*s9 - 880*a1*b3*s5*s7 - 160*a2*b1*s4*s9 + 320*a2*b1*s5*s8 - 160*a2*b1*s6*s7 + 320*a2*b3*s4*s7 - 640*a3*b3*s1*s9 + 1760*a3*b3*s3*s7 - 3200*a1*b1*s5*s10 + 320*a2*b1*s6*s8 - 320*a2*b3*s4*s8 - 6400*a3*b1*s3*s10 + 1600*a1*b1*s7*s9 - 1760*a2*b1*s6*s9 + 880*a2*b3*s4*s9 - 1760*a2*b3*s5*s8 + 880*a2*b3*s6*s7 + 160*a3*b1*s5*s9 + 320*a3*b3*s5*s7 - 3200*a2*b1*s6*s10 + 3200*a2*b3*s4*s10 + 320*a3*b1*s6*s9 - 640*a3*b3*s4*s9 + 320*a3*b3*s5*s8 + 320*a3*b3*s6*s7 + 1600*a2*b1*s8*s9 - 1600*a2*b3*s7*s8 - 880*a3*b3*s5*s9 + 3200*a3*b3*s5*s10 - 1600*a3*b3*s7*s9 + 6400*b1*b3*s1*s10 - 6400*b1*b3*s3*s10 + 3200*b1*b2*s6*s10 - 3200*b2*b3*s4*s10 - 1600*b1*b2*s8*s9 + 1600*b2*b3*s7*s8 - 320*a1*a2*b3*s4^2 + 160*a1*a3*b1*s5^2 - 880*a1*a3*b3*s5^2 + 320*a2*a3*b1*s6^2 + 3520*a1^2*b1*s1*s3 + 3520*a2^2*b1*s2*s3 - 640*a2^2*b3*s1*s2 + 640*a1^2*b1*s1*s6 + 320*a2^2*b1*s2*s5 - 640*a3^2*b3*s1*s3 - 320*a1^2*b1*s4*s5 + 3200*a1*b1^2*s1*s9 + 3200*a1^2*b1*s1*s9 - 1760*a2^2*b3*s2*s5 - 160*a2^2*b1*s4*s6 - 640*a3^2*b3*s3*s4 - 1600*a1*b1^2*s5*s7 - 3200*a1*b3^2*s1*s9 + 3200*a3*b1^2*s3*s7 - 1600*a1^2*b1*s5*s7 + 3200*a2^2*b1*s2*s9 - 3200*a2^2*b3*s2*s7 + 880*a2^2*b3*s4*s6 + 1600*a1*b3^2*s5*s7 + 1600*a2*b1^2*s4*s9 - 3200*a2*b1^2*s5*s8 + 1600*a2*b1^2*s6*s7 - 3200*a3*b3^2*s3*s7 - 3200*a3^2*b3*s3*s7 - 1600*a2^2*b1*s6*s8 + 1600*a2^2*b3*s4*s8 + 320*a3^2*b3*s5*s6 - 1600*a2*b3^2*s4*s9 + 3200*a2*b3^2*s5*s8 - 1600*a2*b3^2*s6*s7 - 1600*a3*b1^2*s5*s9 + 1600*a3*b3^2*s5*s9 + 1600*a3^2*b3*s5*s9 - 1600*a1*a3*b1^2*s5^2 + 1600*a1*a3*b3^2*s5^2 + 1600*a1^2*b1*b3*s5^2 - 1600*a2^2*b1*b3*s4^2 + 1600*a2^2*b1*b3*s6^2 - 1600*a3^2*b1*b3*s5^2 - 3200*a2^2*b1^2*s2*s5 + 3200*a2^2*b3^2*s2*s5 + 1600*a2^2*b1^2*s4*s6 - 1600*a2^2*b3^2*s4*s6 + 1280*a1*a2*b3*s1*s2 - 640*a1*a3*b1*s1*s3 - 320*a1*a2*b1*s1*s6 - 640*a1*a2*b1*s2*s5 + 3520*a1*a2*b1*s3*s4 + 3520*a1*a3*b3*s1*s3 - 1280*a2*a3*b1*s2*s3 - 640*a1*a3*b1*s3*s4 + 160*a1*a2*b1*s4*s5 + 1760*a1*a2*b3*s1*s6 - 320*a2*a3*b1*s3*s4 + 320*a1*a2*b1*s4*s6 + 640*a1*a3*b3*s1*s6 - 1760*a1*a2*b1*s5*s6 + 3200*a1*a2*b3*s1*s8 - 880*a1*a2*b3*s4*s5 - 3200*a1*a3*b1*s3*s7 - 640*a2*a3*b3*s1*s6 + 640*a2*a3*b3*s2*s5 + 1760*a2*a3*b3*s3*s4 + 320*a1*a3*b1*s5*s6 - 320*a1*a3*b3*s4*s5 + 3200*a1*a2*b1*s4*s9 - 1600*a1*a2*b1*s5*s8 - 1600*a1*a2*b1*s6*s7 - 1600*a1*a2*b3*s4*s7 + 3200*a1*a3*b3*s1*s9 - 3200*a2*a3*b1*s3*s8 + 160*a2*a3*b1*s5*s6 + 320*a2*a3*b3*s4*s5 - 320*a2*a3*b3*s4*s6 + 1600*a1*a3*b1*s5*s9 - 1600*a1*a3*b3*s5*s7 - 880*a2*a3*b3*s5*s6 + 1600*a2*a3*b1*s6*s9 + 1600*a2*a3*b3*s4*s9 + 1600*a2*a3*b3*s5*s8 - 3200*a2*a3*b3*s6*s7 - 6400*a1*b1*b3*s3*s7 + 3200*a1*b2*b3*s1*s8 + 6400*a2*b1*b3*s1*s8 - 3200*a2*b1*b2*s2*s9 + 3200*a2*b2*b3*s2*s7 - 1600*a1*b1*b2*s4*s9 - 1600*a1*b1*b2*s5*s8 + 3200*a1*b1*b2*s6*s7 - 1600*a1*b2*b3*s4*s7 - 6400*a2*b1*b3*s3*s8 - 3200*a2*b1*b3*s4*s7 - 3200*a3*b1*b2*s3*s8 + 6400*a3*b1*b3*s1*s9 + 3200*a1*b1*b3*s5*s9 + 1600*a2*b1*b2*s6*s8 - 1600*a2*b2*b3*s4*s8 - 3200*a3*b1*b3*s5*s7 + 3200*a2*b1*b3*s6*s9 + 1600*a3*b1*b2*s6*s9 - 3200*a3*b2*b3*s4*s9 + 1600*a3*b2*b3*s5*s8 + 1600*a3*b2*b3*s6*s7 - 1600*a1*a2*b2*b3*s4^2 + 1600*a2*a3*b1*b2*s6^2 + 6400*a1*a3*b1^2*s1*s3 + 3200*a1*a2*b1^2*s1*s6 - 6400*a1*a3*b3^2*s1*s3 - 1600*a1*a2*b1^2*s4*s5 - 3200*a1*a2*b3^2*s1*s6 + 3200*a2*a3*b1^2*s3*s4 + 1600*a1*a2*b3^2*s4*s5 - 3200*a2*a3*b3^2*s3*s4 - 1600*a2*a3*b1^2*s5*s6 + 1600*a2*a3*b3^2*s5*s6 - 6400*a1^2*b1*b3*s1*s3 + 6400*a2^2*b1*b3*s1*s2 + 3200*a1^2*b1*b2*s1*s6 - 6400*a2^2*b1*b3*s2*s3 + 6400*a3^2*b1*b3*s1*s3 - 1600*a1^2*b1*b2*s4*s5 - 3200*a3^2*b2*b3*s3*s4 + 1600*a3^2*b2*b3*s5*s6 + 6400*a1*a2*b2*b3*s1*s2 - 3200*a1*a2*b1*b2*s2*s5 - 6400*a2*a3*b1*b2*s2*s3 - 6400*a1*a2*b1*b3*s3*s4 - 3200*a1*a3*b1*b2*s3*s4 + 1600*a1*a2*b1*b2*s4*s6 + 3200*a1*a3*b2*b3*s1*s6 + 6400*a2*a3*b1*b3*s1*s6 + 3200*a2*a3*b2*b3*s2*s5 + 3200*a1*a2*b1*b3*s5*s6 + 1600*a1*a3*b1*b2*s5*s6 - 1600*a1*a3*b2*b3*s4*s5 - 3200*a2*a3*b1*b3*s4*s5 - 1600*a2*a3*b2*b3*s4*s6;
                32*s7*s8 - 128*s2*s10 - 968*s3*s10 - 64*s4*s10 - 176*s5*s10 - 32*s1*s10 - 352*s6*s10 + 88*s7*s9 + 176*s8*s9 - 160*a1*s7^2 - 320*a2*s8^2 - 880*a3*s9^2 + 320*b2*s8^2 + 8*s7^2 + 32*s8^2 + 242*s9^2 + 32*a1^2*s4^2 + 242*a1^2*s5^2 + 8*a2^2*s4^2 + 800*a1^2*s7^2 + 242*a2^2*s6^2 + 8*a3^2*s5^2 + 32*a3^2*s6^2 + 800*a2^2*s8^2 + 800*a3^2*s9^2 - 800*b1^2*s7^2 + 1600*b1^2*s9^2 + 800*b2^2*s8^2 + 1600*b3^2*s7^2 - 800*b3^2*s9^2 - 128*a1^2*s1*s2 - 968*a1^2*s1*s3 - 32*a2^2*s1*s2 - 968*a2^2*s2*s3 - 32*a3^2*s1*s3 - 352*a1^2*s1*s6 - 128*a3^2*s2*s3 - 176*a2^2*s2*s5 - 640*a1^2*s1*s8 + 176*a1^2*s4*s5 - 64*a3^2*s3*s4 - 1760*a1^2*s1*s9 - 320*a2^2*s2*s7 - 3200*a1^2*s1*s10 + 320*a1^2*s4*s7 + 88*a2^2*s4*s6 + 880*a1^2*s5*s7 - 1760*a2^2*s2*s9 - 320*a3^2*s3*s7 - 3200*a2^2*s2*s10 + 160*a2^2*s4*s8 - 640*a3^2*s3*s8 + 32*a3^2*s5*s6 + 880*a2^2*s6*s8 - 3200*a3^2*s3*s10 + 160*a3^2*s5*s9 + 320*a3^2*s6*s9 + 3200*b1^2*s1*s10 - 6400*b1^2*s3*s10 - 3200*b2^2*s2*s10 - 6400*b3^2*s1*s10 + 3200*b3^2*s3*s10 + 320*a1^2*b2*s4^2 + 320*a3^2*b2*s6^2 + 64*a1*s1*s8 - 128*a1*s2*s7 + 176*a1*s1*s9 - 968*a1*s3*s7 - 32*a2*s1*s8 + 64*a2*s2*s7 + 640*a1*s1*s10 - 32*a1*s4*s7 + 64*a1*s4*s8 - 88*a1*s5*s7 + 352*a2*s2*s9 - 968*a2*s3*s8 + 16*a2*s4*s7 - 32*a3*s1*s9 + 176*a3*s3*s7 + 176*a1*s4*s9 + 176*a1*s5*s8 - 352*a1*s6*s7 + 1280*a2*s2*s10 - 32*a2*s4*s8 - 128*a3*s2*s9 + 352*a3*s3*s8 + 640*a1*s4*s10 + 484*a1*s5*s9 + 88*a2*s4*s9 - 176*a2*s5*s8 + 88*a2*s6*s7 + 16*a3*s5*s7 + 1760*a1*s5*s10 - 320*a1*s7*s8 + 320*a2*s4*s10 - 176*a2*s6*s8 + 3520*a3*s3*s10 - 64*a3*s4*s9 + 32*a3*s5*s8 + 32*a3*s6*s7 - 880*a1*s7*s9 + 484*a2*s6*s9 - 160*a2*s7*s8 - 88*a3*s5*s9 + 64*a3*s6*s8 + 1760*a2*s6*s10 + 320*a3*s5*s10 - 176*a3*s6*s9 - 880*a2*s8*s9 + 640*a3*s6*s10 - 160*a3*s7*s9 - 320*a3*s8*s9 - 1280*b2*s2*s10 - 320*b2*s4*s10 + 160*b2*s7*s8 - 1760*b2*s6*s10 + 880*b2*s8*s9 + 1600*a1^2*b1^2*s5^2 + 800*a1^2*b2^2*s4^2 - 800*a2^2*b1^2*s4^2 - 800*a1^2*b3^2*s5^2 + 1600*a2^2*b1^2*s6^2 + 1600*a2^2*b3^2*s4^2 - 800*a3^2*b1^2*s5^2 - 800*a2^2*b3^2*s6^2 + 800*a3^2*b2^2*s6^2 + 1600*a3^2*b3^2*s5^2 - 32*a1*a2*s4^2 - 88*a1*a3*s5^2 - 176*a2*a3*s6^2 - 1600*a2*b2*s8^2 + 128*a1*a2*s1*s2 + 352*a1*a3*s1*s3 + 176*a1*a2*s1*s6 + 352*a1*a2*s2*s5 - 968*a1*a2*s3*s4 + 704*a2*a3*s2*s3 + 64*a1*a3*s1*s6 - 128*a1*a3*s2*s5 + 352*a1*a3*s3*s4 + 320*a1*a2*s1*s8 + 640*a1*a2*s2*s7 - 88*a1*a2*s4*s5 - 32*a2*a3*s1*s6 + 64*a2*a3*s2*s5 + 176*a2*a3*s3*s4 - 176*a1*a2*s4*s6 - 32*a1*a3*s4*s5 - 160*a1*a2*s4*s7 + 484*a1*a2*s5*s6 + 320*a1*a3*s1*s9 + 1760*a1*a3*s3*s7 + 64*a1*a3*s4*s6 + 16*a2*a3*s4*s5 - 320*a1*a2*s4*s8 - 176*a1*a3*s5*s6 - 32*a2*a3*s4*s6 - 1760*a1*a2*s4*s9 + 880*a1*a2*s5*s8 + 880*a1*a2*s6*s7 - 160*a1*a3*s5*s7 + 640*a2*a3*s2*s9 + 1760*a2*a3*s3*s8 - 88*a2*a3*s5*s6 - 3200*a1*a2*s4*s10 + 320*a1*a3*s4*s9 - 640*a1*a3*s5*s8 + 320*a1*a3*s6*s7 + 1600*a1*a2*s7*s8 - 880*a1*a3*s5*s9 + 160*a2*a3*s4*s9 + 160*a2*a3*s5*s8 - 320*a2*a3*s6*s7 - 3200*a1*a3*s5*s10 - 320*a2*a3*s6*s8 + 1600*a1*a3*s7*s9 - 880*a2*a3*s6*s9 - 3200*a2*a3*s6*s10 + 1600*a2*a3*s8*s9 + 320*a1*b2*s1*s8 - 1280*a1*b2*s2*s7 + 320*a2*b2*s2*s7 - 160*a1*b2*s4*s7 + 640*a1*b2*s4*s8 + 1760*a2*b2*s2*s9 + 880*a1*b2*s4*s9 + 880*a1*b2*s5*s8 - 1760*a1*b2*s6*s7 + 6400*a2*b2*s2*s10 - 160*a2*b2*s4*s8 - 1280*a3*b2*s2*s9 + 1760*a3*b2*s3*s8 + 3200*a1*b2*s4*s10 - 1600*a1*b2*s7*s8 - 880*a2*b2*s6*s8 - 320*a3*b2*s4*s9 + 160*a3*b2*s5*s8 + 160*a3*b2*s6*s7 + 640*a3*b2*s6*s8 - 880*a3*b2*s6*s9 + 3200*a3*b2*s6*s10 - 1600*a3*b2*s8*s9 + 9600*b1*b3*s5*s10 - 4800*b1*b3*s7*s9 - 160*a1*a2*b2*s4^2 - 880*a2*a3*b2*s6^2 - 1280*a1^2*b2*s1*s2 - 1760*a1^2*b2*s1*s6 - 1280*a3^2*b2*s2*s3 - 6400*a1*b1^2*s3*s7 - 3200*a1*b2^2*s2*s7 + 3200*a2*b1^2*s1*s8 - 3200*a1^2*b2*s1*s8 + 880*a1^2*b2*s4*s5 - 320*a3^2*b2*s3*s4 + 3200*a1*b3^2*s3*s7 - 6400*a2*b1^2*s3*s8 - 1600*a2*b1^2*s4*s7 - 6400*a2*b3^2*s1*s8 + 3200*a3*b1^2*s1*s9 + 1600*a1^2*b2*s4*s7 + 1600*a1*b2^2*s4*s8 + 3200*a1*b1^2*s5*s9 + 3200*a2*b3^2*s3*s8 + 3200*a2*b3^2*s4*s7 - 1600*a3*b1^2*s5*s7 - 3200*a3*b2^2*s2*s9 - 6400*a3*b3^2*s1*s9 - 3200*a3^2*b2*s3*s8 + 160*a3^2*b2*s5*s6 - 1600*a1*b3^2*s5*s9 + 3200*a2*b1^2*s6*s9 + 3200*a3*b3^2*s5*s7 + 1600*a3*b2^2*s6*s8 - 1600*a2*b3^2*s6*s9 + 1600*a3^2*b2*s6*s9 - 6400*a1^2*b1^2*s1*s3 - 3200*a1^2*b2^2*s1*s2 + 3200*a2^2*b1^2*s1*s2 + 3200*a1^2*b3^2*s1*s3 - 6400*a2^2*b1^2*s2*s3 - 6400*a2^2*b3^2*s1*s2 + 3200*a3^2*b1^2*s1*s3 + 3200*a2^2*b3^2*s2*s3 - 3200*a3^2*b2^2*s2*s3 - 6400*a3^2*b3^2*s1*s3 + 640*a1*a2*b2*s1*s2 + 1760*a1*a2*b2*s2*s5 + 3520*a2*a3*b2*s2*s3 + 320*a1*a3*b2*s1*s6 - 1280*a1*a3*b2*s2*s5 + 1760*a1*a3*b2*s3*s4 + 3200*a1*a2*b2*s2*s7 + 320*a2*a3*b2*s2*s5 - 880*a1*a2*b2*s4*s6 - 160*a1*a3*b2*s4*s5 + 640*a1*a3*b2*s4*s6 - 1600*a1*a2*b2*s4*s8 - 880*a1*a3*b2*s5*s6 - 160*a2*a3*b2*s4*s6 + 3200*a2*a3*b2*s2*s9 + 1600*a1*a3*b2*s4*s9 - 3200*a1*a3*b2*s5*s8 + 1600*a1*a3*b2*s6*s7 - 1600*a2*a3*b2*s6*s8 - 9600*a1*b1*b3*s1*s9 + 4800*a1*b1*b3*s5*s7 - 9600*a3*b1*b3*s3*s7 - 4800*a2*b1*b3*s4*s9 + 9600*a2*b1*b3*s5*s8 - 4800*a2*b1*b3*s6*s7 + 4800*a3*b1*b3*s5*s9 + 4800*a1*a3*b1*b3*s5^2 - 6400*a1*a2*b1^2*s3*s4 + 3200*a1*a2*b3^2*s3*s4 - 3200*a1*a3*b2^2*s2*s5 + 3200*a2*a3*b1^2*s1*s6 + 3200*a1*a2*b1^2*s5*s6 - 1600*a2*a3*b1^2*s4*s5 - 6400*a2*a3*b3^2*s1*s6 + 1600*a1*a3*b2^2*s4*s6 - 1600*a1*a2*b3^2*s5*s6 + 3200*a2*a3*b3^2*s4*s5 + 9600*a2^2*b1*b3*s2*s5 - 4800*a2^2*b1*b3*s4*s6 - 19200*a1*a3*b1*b3*s1*s3 - 9600*a1*a2*b1*b3*s1*s6 + 4800*a1*a2*b1*b3*s4*s5 - 9600*a2*a3*b1*b3*s3*s4 + 4800*a2*a3*b1*b3*s5*s6;
                160*b3*s7^2 - 880*b1*s9^2 + 3200*b1^2*s5*s10 - 1600*b1^2*s7*s9 - 3200*b3^2*s5*s10 + 1600*b3^2*s7*s9 - 880*a1^2*b1*s5^2 - 880*a2^2*b1*s6^2 + 160*a2^2*b3*s4^2 + 160*a3^2*b3*s5^2 + 3520*b1*s3*s10 - 640*b3*s1*s10 + 320*b1*s5*s10 + 640*b1*s6*s10 - 160*b1*s7*s9 - 640*b3*s4*s10 - 320*b1*s8*s9 - 1760*b3*s5*s10 + 320*b3*s7*s8 + 880*b3*s7*s9 - 1600*a1*b3*s7^2 + 1600*a3*b1*s9^2 + 1600*b1*b3*s7^2 - 1600*b1*b3*s9^2 - 320*a1*b1*s1*s9 + 3520*a1*b1*s3*s7 + 640*a1*b3*s1*s8 + 160*a1*b1*s5*s7 + 1760*a1*b3*s1*s9 - 640*a2*b1*s2*s9 + 3520*a2*b1*s3*s8 - 640*a2*b3*s1*s8 + 640*a2*b3*s2*s7 - 320*a3*b1*s3*s7 - 320*a1*b1*s4*s9 - 320*a1*b1*s5*s8 + 640*a1*b1*s6*s7 + 6400*a1*b3*s1*s10 - 320*a1*b3*s4*s7 - 640*a3*b1*s3*s8 - 1760*a1*b1*s5*s9 - 880*a1*b3*s5*s7 - 160*a2*b1*s4*s9 + 320*a2*b1*s5*s8 - 160*a2*b1*s6*s7 + 320*a2*b3*s4*s7 - 640*a3*b3*s1*s9 + 1760*a3*b3*s3*s7 - 3200*a1*b1*s5*s10 + 320*a2*b1*s6*s8 - 320*a2*b3*s4*s8 - 6400*a3*b1*s3*s10 + 1600*a1*b1*s7*s9 - 1760*a2*b1*s6*s9 + 880*a2*b3*s4*s9 - 1760*a2*b3*s5*s8 + 880*a2*b3*s6*s7 + 160*a3*b1*s5*s9 + 320*a3*b3*s5*s7 - 3200*a2*b1*s6*s10 + 3200*a2*b3*s4*s10 + 320*a3*b1*s6*s9 - 640*a3*b3*s4*s9 + 320*a3*b3*s5*s8 + 320*a3*b3*s6*s7 + 1600*a2*b1*s8*s9 - 1600*a2*b3*s7*s8 - 880*a3*b3*s5*s9 + 3200*a3*b3*s5*s10 - 1600*a3*b3*s7*s9 - 6400*b1*b3*s1*s10 + 6400*b1*b3*s3*s10 + 3200*b1*b2*s6*s10 - 3200*b2*b3*s4*s10 - 1600*b1*b2*s8*s9 + 1600*b2*b3*s7*s8 - 320*a1*a2*b3*s4^2 + 160*a1*a3*b1*s5^2 - 880*a1*a3*b3*s5^2 + 320*a2*a3*b1*s6^2 + 3520*a1^2*b1*s1*s3 + 3520*a2^2*b1*s2*s3 - 640*a2^2*b3*s1*s2 + 640*a1^2*b1*s1*s6 + 320*a2^2*b1*s2*s5 - 640*a3^2*b3*s1*s3 - 320*a1^2*b1*s4*s5 - 3200*a1*b1^2*s1*s9 + 3200*a1^2*b1*s1*s9 - 1760*a2^2*b3*s2*s5 - 160*a2^2*b1*s4*s6 - 640*a3^2*b3*s3*s4 + 1600*a1*b1^2*s5*s7 + 3200*a1*b3^2*s1*s9 - 3200*a3*b1^2*s3*s7 - 1600*a1^2*b1*s5*s7 + 3200*a2^2*b1*s2*s9 - 3200*a2^2*b3*s2*s7 + 880*a2^2*b3*s4*s6 - 1600*a1*b3^2*s5*s7 - 1600*a2*b1^2*s4*s9 + 3200*a2*b1^2*s5*s8 - 1600*a2*b1^2*s6*s7 + 3200*a3*b3^2*s3*s7 - 3200*a3^2*b3*s3*s7 - 1600*a2^2*b1*s6*s8 + 1600*a2^2*b3*s4*s8 + 320*a3^2*b3*s5*s6 + 1600*a2*b3^2*s4*s9 - 3200*a2*b3^2*s5*s8 + 1600*a2*b3^2*s6*s7 + 1600*a3*b1^2*s5*s9 - 1600*a3*b3^2*s5*s9 + 1600*a3^2*b3*s5*s9 + 1600*a1*a3*b1^2*s5^2 - 1600*a1*a3*b3^2*s5^2 - 1600*a1^2*b1*b3*s5^2 + 1600*a2^2*b1*b3*s4^2 - 1600*a2^2*b1*b3*s6^2 + 1600*a3^2*b1*b3*s5^2 + 3200*a2^2*b1^2*s2*s5 - 3200*a2^2*b3^2*s2*s5 - 1600*a2^2*b1^2*s4*s6 + 1600*a2^2*b3^2*s4*s6 + 1280*a1*a2*b3*s1*s2 - 640*a1*a3*b1*s1*s3 - 320*a1*a2*b1*s1*s6 - 640*a1*a2*b1*s2*s5 + 3520*a1*a2*b1*s3*s4 + 3520*a1*a3*b3*s1*s3 - 1280*a2*a3*b1*s2*s3 - 640*a1*a3*b1*s3*s4 + 160*a1*a2*b1*s4*s5 + 1760*a1*a2*b3*s1*s6 - 320*a2*a3*b1*s3*s4 + 320*a1*a2*b1*s4*s6 + 640*a1*a3*b3*s1*s6 - 1760*a1*a2*b1*s5*s6 + 3200*a1*a2*b3*s1*s8 - 880*a1*a2*b3*s4*s5 - 3200*a1*a3*b1*s3*s7 - 640*a2*a3*b3*s1*s6 + 640*a2*a3*b3*s2*s5 + 1760*a2*a3*b3*s3*s4 + 320*a1*a3*b1*s5*s6 - 320*a1*a3*b3*s4*s5 + 3200*a1*a2*b1*s4*s9 - 1600*a1*a2*b1*s5*s8 - 1600*a1*a2*b1*s6*s7 - 1600*a1*a2*b3*s4*s7 + 3200*a1*a3*b3*s1*s9 - 3200*a2*a3*b1*s3*s8 + 160*a2*a3*b1*s5*s6 + 320*a2*a3*b3*s4*s5 - 320*a2*a3*b3*s4*s6 + 1600*a1*a3*b1*s5*s9 - 1600*a1*a3*b3*s5*s7 - 880*a2*a3*b3*s5*s6 + 1600*a2*a3*b1*s6*s9 + 1600*a2*a3*b3*s4*s9 + 1600*a2*a3*b3*s5*s8 - 3200*a2*a3*b3*s6*s7 + 6400*a1*b1*b3*s3*s7 + 3200*a1*b2*b3*s1*s8 - 6400*a2*b1*b3*s1*s8 - 3200*a2*b1*b2*s2*s9 + 3200*a2*b2*b3*s2*s7 - 1600*a1*b1*b2*s4*s9 - 1600*a1*b1*b2*s5*s8 + 3200*a1*b1*b2*s6*s7 - 1600*a1*b2*b3*s4*s7 + 6400*a2*b1*b3*s3*s8 + 3200*a2*b1*b3*s4*s7 - 3200*a3*b1*b2*s3*s8 - 6400*a3*b1*b3*s1*s9 - 3200*a1*b1*b3*s5*s9 + 1600*a2*b1*b2*s6*s8 - 1600*a2*b2*b3*s4*s8 + 3200*a3*b1*b3*s5*s7 - 3200*a2*b1*b3*s6*s9 + 1600*a3*b1*b2*s6*s9 - 3200*a3*b2*b3*s4*s9 + 1600*a3*b2*b3*s5*s8 + 1600*a3*b2*b3*s6*s7 - 1600*a1*a2*b2*b3*s4^2 + 1600*a2*a3*b1*b2*s6^2 - 6400*a1*a3*b1^2*s1*s3 - 3200*a1*a2*b1^2*s1*s6 + 6400*a1*a3*b3^2*s1*s3 + 1600*a1*a2*b1^2*s4*s5 + 3200*a1*a2*b3^2*s1*s6 - 3200*a2*a3*b1^2*s3*s4 - 1600*a1*a2*b3^2*s4*s5 + 3200*a2*a3*b3^2*s3*s4 + 1600*a2*a3*b1^2*s5*s6 - 1600*a2*a3*b3^2*s5*s6 + 6400*a1^2*b1*b3*s1*s3 - 6400*a2^2*b1*b3*s1*s2 + 3200*a1^2*b1*b2*s1*s6 + 6400*a2^2*b1*b3*s2*s3 - 6400*a3^2*b1*b3*s1*s3 - 1600*a1^2*b1*b2*s4*s5 - 3200*a3^2*b2*b3*s3*s4 + 1600*a3^2*b2*b3*s5*s6 + 6400*a1*a2*b2*b3*s1*s2 - 3200*a1*a2*b1*b2*s2*s5 - 6400*a2*a3*b1*b2*s2*s3 + 6400*a1*a2*b1*b3*s3*s4 - 3200*a1*a3*b1*b2*s3*s4 + 1600*a1*a2*b1*b2*s4*s6 + 3200*a1*a3*b2*b3*s1*s6 - 6400*a2*a3*b1*b3*s1*s6 + 3200*a2*a3*b2*b3*s2*s5 - 3200*a1*a2*b1*b3*s5*s6 + 1600*a1*a3*b1*b2*s5*s6 - 1600*a1*a3*b2*b3*s4*s5 + 3200*a2*a3*b1*b3*s4*s5 - 1600*a2*a3*b2*b3*s4*s6;
                16*s7*s8 - 64*s2*s10 - 484*s3*s10 - 32*s4*s10 - 88*s5*s10 - 16*s1*s10 - 176*s6*s10 + 44*s7*s9 + 88*s8*s9 - 80*a1*s7^2 - 160*a2*s8^2 - 440*a3*s9^2 + 80*b1*s7^2 + 160*b2*s8^2 + 440*b3*s9^2 + 4*s7^2 + 16*s8^2 + 121*s9^2 + 16*a1^2*s4^2 + 121*a1^2*s5^2 + 4*a2^2*s4^2 + 400*a1^2*s7^2 + 121*a2^2*s6^2 + 4*a3^2*s5^2 + 16*a3^2*s6^2 + 400*a2^2*s8^2 + 400*a3^2*s9^2 + 400*b1^2*s7^2 + 400*b2^2*s8^2 + 400*b3^2*s9^2 - 64*a1^2*s1*s2 - 484*a1^2*s1*s3 - 16*a2^2*s1*s2 - 484*a2^2*s2*s3 - 16*a3^2*s1*s3 - 176*a1^2*s1*s6 - 64*a3^2*s2*s3 - 88*a2^2*s2*s5 - 320*a1^2*s1*s8 + 88*a1^2*s4*s5 - 32*a3^2*s3*s4 - 880*a1^2*s1*s9 - 160*a2^2*s2*s7 - 1600*a1^2*s1*s10 + 160*a1^2*s4*s7 + 44*a2^2*s4*s6 + 440*a1^2*s5*s7 - 880*a2^2*s2*s9 - 160*a3^2*s3*s7 - 1600*a2^2*s2*s10 + 80*a2^2*s4*s8 - 320*a3^2*s3*s8 + 16*a3^2*s5*s6 + 440*a2^2*s6*s8 - 1600*a3^2*s3*s10 + 80*a3^2*s5*s9 + 160*a3^2*s6*s9 - 1600*b1^2*s1*s10 - 1600*b2^2*s2*s10 - 1600*b3^2*s3*s10 + 160*a1^2*b2*s4^2 + 80*a2^2*b1*s4^2 + 440*a1^2*b3*s5^2 + 80*a3^2*b1*s5^2 + 440*a2^2*b3*s6^2 + 160*a3^2*b2*s6^2 + 32*a1*s1*s8 - 64*a1*s2*s7 + 88*a1*s1*s9 - 484*a1*s3*s7 - 16*a2*s1*s8 + 32*a2*s2*s7 + 320*a1*s1*s10 - 16*a1*s4*s7 + 32*a1*s4*s8 - 44*a1*s5*s7 + 176*a2*s2*s9 - 484*a2*s3*s8 + 8*a2*s4*s7 - 16*a3*s1*s9 + 88*a3*s3*s7 + 88*a1*s4*s9 + 88*a1*s5*s8 - 176*a1*s6*s7 + 640*a2*s2*s10 - 16*a2*s4*s8 - 64*a3*s2*s9 + 176*a3*s3*s8 + 320*a1*s4*s10 + 242*a1*s5*s9 + 44*a2*s4*s9 - 88*a2*s5*s8 + 44*a2*s6*s7 + 8*a3*s5*s7 + 880*a1*s5*s10 - 160*a1*s7*s8 + 160*a2*s4*s10 - 88*a2*s6*s8 + 1760*a3*s3*s10 - 32*a3*s4*s9 + 16*a3*s5*s8 + 16*a3*s6*s7 - 440*a1*s7*s9 + 242*a2*s6*s9 - 80*a2*s7*s8 - 44*a3*s5*s9 + 32*a3*s6*s8 + 880*a2*s6*s10 + 160*a3*s5*s10 - 88*a3*s6*s9 - 440*a2*s8*s9 + 320*a3*s6*s10 - 80*a3*s7*s9 - 160*a3*s8*s9 - 320*b1*s1*s10 - 640*b2*s2*s10 - 320*b1*s4*s10 - 880*b1*s5*s10 + 160*b1*s7*s8 - 160*b2*s4*s10 - 1760*b3*s3*s10 + 440*b1*s7*s9 + 80*b2*s7*s8 - 880*b2*s6*s10 - 160*b3*s5*s10 + 440*b2*s8*s9 - 320*b3*s6*s10 + 80*b3*s7*s9 + 160*b3*s8*s9 + 400*a1^2*b2^2*s4^2 + 400*a2^2*b1^2*s4^2 + 400*a1^2*b3^2*s5^2 + 400*a3^2*b1^2*s5^2 + 400*a2^2*b3^2*s6^2 + 400*a3^2*b2^2*s6^2 - 16*a1*a2*s4^2 - 44*a1*a3*s5^2 - 88*a2*a3*s6^2 - 800*a1*b1*s7^2 - 800*a2*b2*s8^2 - 800*a3*b3*s9^2 + 64*a1*a2*s1*s2 + 176*a1*a3*s1*s3 + 88*a1*a2*s1*s6 + 176*a1*a2*s2*s5 - 484*a1*a2*s3*s4 + 352*a2*a3*s2*s3 + 32*a1*a3*s1*s6 - 64*a1*a3*s2*s5 + 176*a1*a3*s3*s4 + 160*a1*a2*s1*s8 + 320*a1*a2*s2*s7 - 44*a1*a2*s4*s5 - 16*a2*a3*s1*s6 + 32*a2*a3*s2*s5 + 88*a2*a3*s3*s4 - 88*a1*a2*s4*s6 - 16*a1*a3*s4*s5 - 80*a1*a2*s4*s7 + 242*a1*a2*s5*s6 + 160*a1*a3*s1*s9 + 880*a1*a3*s3*s7 + 32*a1*a3*s4*s6 + 8*a2*a3*s4*s5 - 160*a1*a2*s4*s8 - 88*a1*a3*s5*s6 - 16*a2*a3*s4*s6 - 880*a1*a2*s4*s9 + 440*a1*a2*s5*s8 + 440*a1*a2*s6*s7 - 80*a1*a3*s5*s7 + 320*a2*a3*s2*s9 + 880*a2*a3*s3*s8 - 44*a2*a3*s5*s6 - 1600*a1*a2*s4*s10 + 160*a1*a3*s4*s9 - 320*a1*a3*s5*s8 + 160*a1*a3*s6*s7 + 800*a1*a2*s7*s8 - 440*a1*a3*s5*s9 + 80*a2*a3*s4*s9 + 80*a2*a3*s5*s8 - 160*a2*a3*s6*s7 - 1600*a1*a3*s5*s10 - 160*a2*a3*s6*s8 + 800*a1*a3*s7*s9 - 440*a2*a3*s6*s9 - 1600*a2*a3*s6*s10 + 800*a2*a3*s8*s9 + 320*a1*b1*s1*s8 + 880*a1*b1*s1*s9 + 160*a1*b2*s1*s8 - 640*a1*b2*s2*s7 - 320*a2*b1*s1*s8 + 320*a2*b1*s2*s7 + 3200*a1*b1*s1*s10 - 160*a1*b1*s4*s7 + 160*a2*b2*s2*s7 - 440*a1*b1*s5*s7 - 80*a1*b2*s4*s7 + 160*a1*b3*s1*s9 - 1760*a1*b3*s3*s7 + 160*a2*b1*s4*s7 - 320*a3*b1*s1*s9 + 880*a3*b1*s3*s7 + 320*a1*b2*s4*s8 - 160*a2*b1*s4*s8 + 880*a2*b2*s2*s9 + 440*a1*b2*s4*s9 + 440*a1*b2*s5*s8 - 880*a1*b2*s6*s7 - 80*a1*b3*s5*s7 + 440*a2*b1*s4*s9 - 880*a2*b1*s5*s8 + 440*a2*b1*s6*s7 + 3200*a2*b2*s2*s10 - 80*a2*b2*s4*s8 + 320*a2*b3*s2*s9 - 1760*a2*b3*s3*s8 + 160*a3*b1*s5*s7 - 640*a3*b2*s2*s9 + 880*a3*b2*s3*s8 + 160*a3*b3*s3*s7 + 1600*a1*b2*s4*s10 + 160*a1*b3*s4*s9 + 160*a1*b3*s5*s8 - 320*a1*b3*s6*s7 + 1600*a2*b1*s4*s10 - 320*a3*b1*s4*s9 + 160*a3*b1*s5*s8 + 160*a3*b1*s6*s7 + 320*a3*b3*s3*s8 - 800*a1*b2*s7*s8 + 880*a1*b3*s5*s9 - 800*a2*b1*s7*s8 - 440*a2*b2*s6*s8 + 80*a2*b3*s4*s9 - 160*a2*b3*s5*s8 + 80*a2*b3*s6*s7 - 440*a3*b1*s5*s9 - 160*a3*b2*s4*s9 + 80*a3*b2*s5*s8 + 80*a3*b2*s6*s7 + 1600*a1*b3*s5*s10 - 160*a2*b3*s6*s8 + 1600*a3*b1*s5*s10 + 320*a3*b2*s6*s8 + 3200*a3*b3*s3*s10 - 800*a1*b3*s7*s9 + 880*a2*b3*s6*s9 - 800*a3*b1*s7*s9 - 440*a3*b2*s6*s9 - 80*a3*b3*s5*s9 + 1600*a2*b3*s6*s10 + 1600*a3*b2*s6*s10 - 160*a3*b3*s6*s9 - 800*a2*b3*s8*s9 - 800*a3*b2*s8*s9 - 1600*b1*b2*s4*s10 + 800*b1*b2*s7*s8 - 1600*b1*b3*s5*s10 + 800*b1*b3*s7*s9 - 1600*b2*b3*s6*s10 + 800*b2*b3*s8*s9 - 160*a1*a2*b1*s4^2 - 80*a1*a2*b2*s4^2 - 440*a1*a3*b1*s5^2 - 80*a1*a3*b3*s5^2 - 440*a2*a3*b2*s6^2 - 160*a2*a3*b3*s6^2 - 640*a1^2*b2*s1*s2 - 320*a2^2*b1*s1*s2 - 1760*a1^2*b3*s1*s3 - 320*a3^2*b1*s1*s3 - 880*a1^2*b2*s1*s6 - 880*a2^2*b1*s2*s5 - 1760*a2^2*b3*s2*s3 - 640*a3^2*b2*s2*s3 - 320*a1^2*b3*s1*s6 - 320*a3^2*b1*s3*s4 - 1600*a1*b2^2*s2*s7 - 1600*a2*b1^2*s1*s8 - 1600*a1^2*b2*s1*s8 + 440*a1^2*b2*s4*s5 - 1600*a2^2*b1*s2*s7 - 160*a2^2*b3*s2*s5 - 160*a3^2*b2*s3*s4 + 160*a1^2*b3*s4*s5 + 440*a2^2*b1*s4*s6 - 1600*a1*b3^2*s3*s7 + 800*a2*b1^2*s4*s7 - 1600*a3*b1^2*s1*s9 + 800*a1^2*b2*s4*s7 - 1600*a1^2*b3*s1*s9 - 1600*a3^2*b1*s3*s7 + 800*a1*b2^2*s4*s8 + 800*a2^2*b1*s4*s8 + 80*a2^2*b3*s4*s6 + 160*a3^2*b1*s5*s6 - 1600*a2*b3^2*s3*s8 + 800*a3*b1^2*s5*s7 - 1600*a3*b2^2*s2*s9 + 800*a1^2*b3*s5*s7 - 1600*a2^2*b3*s2*s9 - 1600*a3^2*b2*s3*s8 + 80*a3^2*b2*s5*s6 + 800*a1*b3^2*s5*s9 + 800*a3^2*b1*s5*s9 + 800*a3*b2^2*s6*s8 + 800*a2^2*b3*s6*s8 + 800*a2*b3^2*s6*s9 + 800*a3^2*b2*s6*s9 - 1600*a1^2*b2^2*s1*s2 - 1600*a2^2*b1^2*s1*s2 - 1600*a1^2*b3^2*s1*s3 - 1600*a3^2*b1^2*s1*s3 - 1600*a2^2*b3^2*s2*s3 - 1600*a3^2*b2^2*s2*s3 + 640*a1*a2*b1*s1*s2 + 320*a1*a2*b2*s1*s2 + 1760*a1*a3*b1*s1*s3 + 880*a1*a2*b1*s1*s6 + 320*a1*a3*b3*s1*s3 + 880*a1*a2*b2*s2*s5 + 320*a1*a3*b1*s1*s6 + 1760*a2*a3*b2*s2*s3 + 1600*a1*a2*b1*s1*s8 - 440*a1*a2*b1*s4*s5 + 160*a1*a2*b3*s1*s6 + 320*a1*a2*b3*s2*s5 - 1760*a1*a2*b3*s3*s4 + 160*a1*a3*b2*s1*s6 - 640*a1*a3*b2*s2*s5 + 880*a1*a3*b2*s3*s4 - 320*a2*a3*b1*s1*s6 + 320*a2*a3*b1*s2*s5 + 880*a2*a3*b1*s3*s4 + 640*a2*a3*b3*s2*s3 + 1600*a1*a2*b2*s2*s7 - 160*a1*a3*b1*s4*s5 + 320*a1*a3*b3*s3*s4 + 160*a2*a3*b2*s2*s5 - 800*a1*a2*b1*s4*s7 - 440*a1*a2*b2*s4*s6 - 80*a1*a2*b3*s4*s5 + 1600*a1*a3*b1*s1*s9 - 80*a1*a3*b2*s4*s5 + 160*a2*a3*b1*s4*s5 + 160*a2*a3*b3*s3*s4 - 160*a1*a2*b3*s4*s6 + 320*a1*a3*b2*s4*s6 - 160*a2*a3*b1*s4*s6 - 800*a1*a2*b2*s4*s8 + 880*a1*a2*b3*s5*s6 - 800*a1*a3*b1*s5*s7 - 440*a1*a3*b2*s5*s6 + 1600*a1*a3*b3*s3*s7 - 440*a2*a3*b1*s5*s6 - 80*a2*a3*b2*s4*s6 - 160*a1*a3*b3*s5*s6 + 1600*a2*a3*b2*s2*s9 - 1600*a1*a2*b3*s4*s9 + 800*a1*a2*b3*s5*s8 + 800*a1*a2*b3*s6*s7 + 800*a1*a3*b2*s4*s9 - 1600*a1*a3*b2*s5*s8 + 800*a1*a3*b2*s6*s7 + 800*a2*a3*b1*s4*s9 + 800*a2*a3*b1*s5*s8 - 1600*a2*a3*b1*s6*s7 + 1600*a2*a3*b3*s3*s8 - 80*a2*a3*b3*s5*s6 - 800*a1*a3*b3*s5*s9 - 800*a2*a3*b2*s6*s8 - 800*a2*a3*b3*s6*s9 + 1600*a1*b1*b2*s1*s8 + 1600*a2*b1*b2*s2*s7 - 800*a1*b1*b2*s4*s7 + 1600*a1*b1*b3*s1*s9 - 800*a1*b1*b3*s5*s7 - 800*a2*b1*b2*s4*s8 + 1600*a3*b1*b3*s3*s7 + 1600*a2*b2*b3*s2*s9 + 800*a1*b2*b3*s4*s9 + 800*a1*b2*b3*s5*s8 - 1600*a1*b2*b3*s6*s7 + 800*a2*b1*b3*s4*s9 - 1600*a2*b1*b3*s5*s8 + 800*a2*b1*b3*s6*s7 - 1600*a3*b1*b2*s4*s9 + 800*a3*b1*b2*s5*s8 + 800*a3*b1*b2*s6*s7 + 1600*a3*b2*b3*s3*s8 - 800*a2*b2*b3*s6*s8 - 800*a3*b1*b3*s5*s9 - 800*a3*b2*b3*s6*s9 - 800*a1*a2*b1*b2*s4^2 - 800*a1*a3*b1*b3*s5^2 - 800*a2*a3*b2*b3*s6^2 - 1600*a1*a2*b3^2*s3*s4 - 1600*a1*a3*b2^2*s2*s5 - 1600*a2*a3*b1^2*s1*s6 + 800*a2*a3*b1^2*s4*s5 + 800*a1*a3*b2^2*s4*s6 + 800*a1*a2*b3^2*s5*s6 - 1600*a1^2*b2*b3*s1*s6 - 1600*a2^2*b1*b3*s2*s5 - 1600*a3^2*b1*b2*s3*s4 + 800*a1^2*b2*b3*s4*s5 + 800*a2^2*b1*b3*s4*s6 + 800*a3^2*b1*b2*s5*s6 + 3200*a1*a2*b1*b2*s1*s2 + 3200*a1*a3*b1*b3*s1*s3 + 1600*a1*a2*b1*b3*s1*s6 + 1600*a1*a3*b1*b2*s1*s6 + 1600*a1*a2*b2*b3*s2*s5 + 1600*a2*a3*b1*b2*s2*s5 + 3200*a2*a3*b2*b3*s2*s3 - 800*a1*a2*b1*b3*s4*s5 - 800*a1*a3*b1*b2*s4*s5 + 1600*a1*a3*b2*b3*s3*s4 + 1600*a2*a3*b1*b3*s3*s4 - 800*a1*a2*b2*b3*s4*s6 - 800*a2*a3*b1*b2*s4*s6 - 800*a1*a3*b2*b3*s5*s6 - 800*a2*a3*b1*b3*s5*s6];
            
            u_value = unique(double((roots(u_coeff))));
            u_value = u_value(imag(u_value)==0);
            u_value(u_value <u_range(1)) = [];u_value(u_value > u_range(2)) = [];
            
            if ~isempty(u_value)
                for i = 1:size(u_value,1)
                    v_coeff = subs(v_coeff_u,u_value(i));
                    v_value = double(-v_coeff(2)/(2*v_coeff(1)));
                    v_value = v_value(imag(v_value)==0);
                    v_value(v_value <0) = [];v_value(v_value > 1) = [];
                    %% find the corresponding pose by finding the same unit vector
                    if ~isempty(v_value)
                        for ii = 1:size(v_value,2)
                            %% check if out of surface boundary
                            tmp_val = uv_equ(u_value(i),v_value(ii));
                            if tmp_val(1) <= obj.surface_bound{surf_ind}(2) && tmp_val(1) >= obj.surface_bound{surf_ind}(1) && ...
                                    tmp_val(2) <= obj.surface_bound{surf_ind}(4) && tmp_val(2) >= obj.surface_bound{surf_ind}(3) &&...
                                    tmp_val(3) <= obj.surface_bound{surf_ind}(6) && tmp_val(3) >= obj.surface_bound{surf_ind}(5)
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
        function [q_intersected,intersected_pts] = Curve2QuadSurfIntersection(obj,QuadSurfCoeff,r_GA_i,AttPts,t_equ,q_begin,q_end,u_range,surf_ind)
            q_intersected = [];intersected_pts = [];
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
            
            a1 = AttPts(1,1);
            a2 = AttPts(1,2);
            a3 = AttPts(1,3);
            b1 = r_GA_i(1);
            b2 = r_GA_i(2);
            b3 = r_GA_i(3);
            
             %%%%%%% A*t^2 + B*t + C = 0, coefficient of t is shown below in
             %%%%%%% the form of t_coeff = [A;B;C;...] 
            for i = 1:3
                if i == 1
                    t_coeff = [4*s1 + 16*s2 + 961*s3 + 8*s4 + 62*s5 + 124*s6 + 40*s7 + 80*s8 + 620*s9 + 400*s10 - 80*a1*s1 - 160*a2*s2 - 80*a1*s4 - 620*a1*s5 - 40*a2*s4 - 1240*a3*s3 - 400*a1*s7 - 620*a2*s6 - 40*a3*s5 - 80*a3*s6 - 400*a2*s8 - 400*a3*s9 - 80*b1*s1 + 160*b2*s2 - 80*b1*s4 - 620*b1*s5 + 40*b2*s4 - 1240*b3*s3 - 400*b1*s7 + 620*b2*s6 - 40*b3*s5 - 80*b3*s6 + 400*b2*s8 - 400*b3*s9 + 400*a1^2*s1 + 400*a2^2*s2 + 400*a3^2*s3 + 400*b1^2*s1 + 400*b2^2*s2 + 400*b3^2*s3 + 400*a1*a2*s4 + 400*a1*a3*s5 + 400*a2*a3*s6 + 800*a1*b1*s1 - 800*a2*b2*s2 - 400*a1*b2*s4 + 400*a2*b1*s4 + 400*a1*b3*s5 + 400*a3*b1*s5 + 800*a3*b3*s3 + 400*a2*b3*s6 - 400*a3*b2*s6 - 400*b1*b2*s4 + 400*b1*b3*s5 - 400*b2*b3*s6;
                        160*b3*s1 - 2480*b1*s3 - 80*b1*s5 - 160*b1*s6 + 160*b3*s4 + 1240*b3*s5 - 800*b1*s9 + 800*b3*s7 + 800*b1^2*s5 - 800*b3^2*s5 - 1600*a1*b3*s1 + 800*a1*b1*s5 + 1600*a3*b1*s3 + 800*a2*b1*s6 - 800*a2*b3*s4 - 800*a3*b3*s5 - 1600*b1*b3*s1 + 1600*b1*b3*s3 - 800*b1*b2*s6 + 800*b2*b3*s4;
                        8*s1 + 32*s2 + 1922*s3 + 16*s4 + 124*s5 + 248*s6 + 80*s7 + 160*s8 + 1240*s9 + 800*s10 - 160*a1*s1 - 320*a2*s2 - 160*a1*s4 - 1240*a1*s5 - 80*a2*s4 - 2480*a3*s3 - 800*a1*s7 - 1240*a2*s6 - 80*a3*s5 - 160*a3*s6 - 800*a2*s8 - 800*a3*s9 + 320*b2*s2 + 80*b2*s4 + 1240*b2*s6 + 800*b2*s8 + 800*a1^2*s1 + 800*a2^2*s2 + 800*a3^2*s3 - 800*b1^2*s1 + 1600*b1^2*s3 + 800*b2^2*s2 + 1600*b3^2*s1 - 800*b3^2*s3 + 800*a1*a2*s4 + 800*a1*a3*s5 + 800*a2*a3*s6 - 1600*a2*b2*s2 - 800*a1*b2*s4 - 800*a3*b2*s6 - 2400*b1*b3*s5;
                        160*b3*s1 - 2480*b1*s3 - 80*b1*s5 - 160*b1*s6 + 160*b3*s4 + 1240*b3*s5 - 800*b1*s9 + 800*b3*s7 - 800*b1^2*s5 + 800*b3^2*s5 - 1600*a1*b3*s1 + 800*a1*b1*s5 + 1600*a3*b1*s3 + 800*a2*b1*s6 - 800*a2*b3*s4 - 800*a3*b3*s5 + 1600*b1*b3*s1 - 1600*b1*b3*s3 - 800*b1*b2*s6 + 800*b2*b3*s4;
                        4*s1 + 16*s2 + 961*s3 + 8*s4 + 62*s5 + 124*s6 + 40*s7 + 80*s8 + 620*s9 + 400*s10 - 80*a1*s1 - 160*a2*s2 - 80*a1*s4 - 620*a1*s5 - 40*a2*s4 - 1240*a3*s3 - 400*a1*s7 - 620*a2*s6 - 40*a3*s5 - 80*a3*s6 - 400*a2*s8 - 400*a3*s9 + 80*b1*s1 + 160*b2*s2 + 80*b1*s4 + 620*b1*s5 + 40*b2*s4 + 1240*b3*s3 + 400*b1*s7 + 620*b2*s6 + 40*b3*s5 + 80*b3*s6 + 400*b2*s8 + 400*b3*s9 + 400*a1^2*s1 + 400*a2^2*s2 + 400*a3^2*s3 + 400*b1^2*s1 + 400*b2^2*s2 + 400*b3^2*s3 + 400*a1*a2*s4 + 400*a1*a3*s5 + 400*a2*a3*s6 - 800*a1*b1*s1 - 800*a2*b2*s2 - 400*a1*b2*s4 - 400*a2*b1*s4 - 400*a1*b3*s5 - 400*a3*b1*s5 - 800*a3*b3*s3 - 400*a2*b3*s6 - 400*a3*b2*s6 + 400*b1*b2*s4 + 400*b1*b3*s5 + 400*b2*b3*s6
                        ];
                elseif i == 2
                    c1 = AttPts(3,1);
                    c2 = AttPts(3,2);
                    c3 = AttPts(3,3);
                    
                    t_coeff = [25*a1^2 - 50*a1*c1 + 25*a2^2 - 50*a2*c2 + 25*a3^2 - 50*a3*c3 + 25*c1^2 + 25*c2^2 + 25*c3^2;
                        25*a1 + 25*a2 + 25*a3 - 25*c1 - 25*c2 - 25*c3 + 50*a1*c1 + 50*a2*c2 + 50*a3*c3 - 50*a1^2 - 50*a2^2 - 50*a3^2;
                        25*a1^2 - 25*a1 + 25*a2^2 - 25*a2 + 25*a3^2 - 25*a3 + 18];
                    u_range = [0 1];
                else
                    c1 = AttPts(2,1);
                    c2 = AttPts(2,2);
                    c3 = AttPts(2,3);
                    
                    t_coeff = [25*a1^2 - 50*a1*c1 + 25*a2^2 - 50*a2*c2 + 25*a3^2 - 50*a3*c3 + 25*c1^2 + 25*c2^2 + 25*c3^2;
                        25*a1 + 25*a2 + 25*a3 - 25*c1 - 25*c2 - 25*c3 + 50*a1*c1 + 50*a2*c2 + 50*a3*c3 - 50*a1^2 - 50*a2^2 - 50*a3^2;
                        25*a1^2 - 25*a1 + 25*a2^2 - 25*a2 + 25*a3^2 - 25*a3 + 18];
                    u_range = [0 1];
                end
                
                
                t_ans(1) = (-t_coeff(2) + sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                t_ans(2) = (-t_coeff(2) - sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                
                t_ans = t_ans(imag(t_ans)==0);
                t_ans(t_ans < u_range(1)) = [];t_ans(t_ans > u_range(2)) = [];
                t_ans = double(t_ans);
                if ~isempty(t_ans)
                    for ii = 1:size(t_ans,2)
                        %% finding the corresponding poses of intersection and
                        %% check if out of surface boundary
                        tmp_val = double(t_equ{i}(t_ans(ii)));
                        if tmp_val(1) <= obj.surface_bound{surf_ind}(2) && tmp_val(1) >= obj.surface_bound{surf_ind}(1) && ...
                                tmp_val(2) <= obj.surface_bound{surf_ind}(4) && tmp_val(2) >= obj.surface_bound{surf_ind}(3) &&...
                                tmp_val(3) <= obj.surface_bound{surf_ind}(6) && tmp_val(3) >= obj.surface_bound{surf_ind}(5)
                            intersected_pts = [intersected_pts, tmp_val];
                            q_intersected = [q_intersected,(q_end - q_begin)*t_ans(ii) + q_begin];
                        end
                    end
                end
            end
        end
        
        %% function to calculate the intersected poses between quad-surface and cable segment bounded surface
        function [q_intersected,intersected_pts] = ParametericSurfaceIntersectionTranslation(obj,QuadSurfCoeff,AttPts,uv_equ,q_begin,q_end,surf_ind)
            intersected_pts = [];q_intersected = [];
            %%
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
            
            %%            
            %%%%%%% A*v^2 + B*v + C = 0, coefficient of v is shown below in
            %%%%%%% the form of v_coeff = [A;B;C]
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
                    %%%%%%% find v using -B/(2*A*C) since the B^2 - 4*A*C is zero
                    v_coeff = v_coeff_u(u_value(i));
                    v_value = -v_coeff(2)/(2*v_coeff(1));
                    v_value = v_value(imag(v_value)==0);
                    v_value(v_value <0) = [];v_value(v_value > 1) = [];
                    %% find the corresponding pose by finding the same unit vector
                    if ~isempty(v_value)
                        for ii = 1:size(v_value,2)
                            %% check if out of surface boundary
                            tmp_val = uv_equ(u_value(i),v_value(ii));
                            if tmp_val(1) <= obj.surface_bound{surf_ind}(2) && tmp_val(1) >= obj.surface_bound{surf_ind}(1) && ...
                                    tmp_val(2) <= obj.surface_bound{surf_ind}(4) && tmp_val(2) >= obj.surface_bound{surf_ind}(3) &&...
                                    tmp_val(3) <= obj.surface_bound{surf_ind}(6) && tmp_val(3) >= obj.surface_bound{surf_ind}(5)
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
        function [q_intersected,intersected_pts] = Edges2QuadSurfIntersection(obj,QuadSurfCoeff,AttPts,q_begin,q_end,surf_ind)
            
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
            
            q_intersected = [];intersected_pts = [];
            att_sequence = [3 2; 3 1; 2 1 ];
            
            for i = 1:3
                b1 = AttPts(att_sequence(i,1),1);
                b2 = AttPts(att_sequence(i,1),2);
                b3 = AttPts(att_sequence(i,1),3);
                
                c1 = AttPts(att_sequence(i,2),1);
                c2 = AttPts(att_sequence(i,2),2);
                c3 = AttPts(att_sequence(i,2),3);
                
                %%%%%%% A*t^2 + B*t + C = 0, coefficient of t is shown below in
                %%%%%%% the form of t_coeff = [A;B;C]
                t_coeff = [s1*b1^2 + s4*b1*b2 + s5*b1*b3 - 2*s1*b1*c1 - s4*b1*c2 - s5*b1*c3 + s2*b2^2 + s6*b2*b3 - s4*b2*c1 - 2*s2*b2*c2 - s6*b2*c3 + s3*b3^2 - s5*b3*c1 - s6*b3*c2 - 2*s3*b3*c3 + s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s2*c2^2 + s6*c2*c3 + s3*c3^2;
                    b1*s7 + b2*s8 + b3*s9 - c1*s7 - c2*s8 - c3*s9 - 2*c1^2*s1 - 2*c2^2*s2 - 2*c3^2*s3 + 2*b1*c1*s1 + 2*b2*c2*s2 + b1*c2*s4 + b2*c1*s4 + b1*c3*s5 + b3*c1*s5 + 2*b3*c3*s3 + b2*c3*s6 + b3*c2*s6 - 2*c1*c2*s4 - 2*c1*c3*s5 - 2*c2*c3*s6;
                    s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s7*c1 + s2*c2^2 + s6*c2*c3 + s8*c2 + s3*c3^2 + s9*c3 + s10];
                %%%%%%% then the solution of A*t^2 + B*t + C = 0 is given below
                
                t_ans(1,:) = (-t_coeff(2) + sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                t_ans(2,:) = (-t_coeff(2) - sqrt(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3)))/(2*t_coeff(1));
                t_ans = unique(t_ans);
                t_ans = t_ans(imag(t_ans)==0);
                t_ans(t_ans <0) = [];t_ans(t_ans > 1) = [];
                if ~isempty(t_ans)
                    for ii = 1:size(t_ans,1)
                        %% check if out of surface boundary
                        tmp_val =  [c1 + t_ans(ii)*(b1 - c1);
                            c2 + t_ans(ii)*(b2 - c2);
                            c3 + t_ans(ii)*(b3 - c3)];
                        
                        if tmp_val(1) <= obj.surface_bound{surf_ind}(2) && tmp_val(1) >= obj.surface_bound{surf_ind}(1) && ...
                                tmp_val(2) <= obj.surface_bound{surf_ind}(4) && tmp_val(2) >= obj.surface_bound{surf_ind}(3) &&...
                                tmp_val(3) <= obj.surface_bound{surf_ind}(6) && tmp_val(3) >= obj.surface_bound{surf_ind}(5)
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
                
                t_coeff = [s1*b1^2 + s4*b1*b2 + s5*b1*b3 - 2*s1*b1*c1 - s4*b1*c2 - s5*b1*c3 + s2*b2^2 + s6*b2*b3 - s4*b2*c1 - 2*s2*b2*c2 - s6*b2*c3 + s3*b3^2 - s5*b3*c1 - s6*b3*c2 - 2*s3*b3*c3 + s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s2*c2^2 + s6*c2*c3 + s3*c3^2;
                    b1*s7 + b2*s8 + b3*s9 - c1*s7 - c2*s8 - c3*s9 - 2*c1^2*s1 - 2*c2^2*s2 - 2*c3^2*s3 + 2*b1*c1*s1 + 2*b2*c2*s2 + b1*c2*s4 + b2*c1*s4 + b1*c3*s5 + b3*c1*s5 + 2*b3*c3*s3 + b2*c3*s6 + b3*c2*s6 - 2*c1*c2*s4 - 2*c1*c3*s5 - 2*c2*c3*s6;
                    s1*c1^2 + s4*c1*c2 + s5*c1*c3 + s7*c1 + s2*c2^2 + s6*c2*c3 + s8*c2 + s3*c3^2 + s9*c3 + s10];
                
                t_condition = round(t_coeff(2)^2 - 4*t_coeff(1)*t_coeff(3),5);
                
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
        function Q  = GetQuadSurfCoeff(~,QuadSurf)
            syms x y z;
            Q = zeros(1,10);
            [coeff_f,var_f] = coeffs(QuadSurf(x,y,z));
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


