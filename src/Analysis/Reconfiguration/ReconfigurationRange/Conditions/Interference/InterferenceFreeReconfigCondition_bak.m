% Class to compute
% Author        : Paul Cheng
% Created       : 2020
% Description   :

classdef InterferenceFreeReconfigCondition < ReconfigConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 9;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = ReconfigConditionType.INTERFERENCE_CABLE_OBSTACLE;
        
    end
    properties (SetAccess = protected)
        model
        obstacle
        obstacle_hull = [];
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeReconfigCondition(model, obstacle)
            w.model = model;
            w.obstacle = obstacle;
            

        end
        
        function feasibleInterval =  evaluateFunction(obj, reconfig_element)
            feasibleInterval = cell(obj.model.numCables,1);
            q_zero = zeros(size(obj.model.q));
            
            if obj.model.modelMode == ModelModeType.COMPILED
                cable_attachment = obj.model.cableModel.compiled_r_OAs_fn(reconfig_element.q ,q_zero,q_zero,q_zero);
            else
                obj.model.update(reconfig_element.q ,q_zero,q_zero,q_zero);
                cable_attachment = obj.model.cableModel.r_OAs;
%                 for i = 1:obj.model.numCables
%                 attachment_point(:,i) = obj.model.cableModel.cables{i}.attachments{1}.constantPulleyPoint; 
%                 end
            end
            
            for cable_id = 1:obj.model.numCables
                 if norm(reconfig_element.OA_begin(:,cable_id) - reconfig_element.OA_end(:,cable_id)) ~= 0
               
                P_int = []; OA_u = [];P_index = [];
                OB = cable_attachment(4:6,cable_id);
                OA = [reconfig_element.OA_end(:,cable_id),reconfig_element.OA_begin(:,cable_id)];
                for surf_id = 1:obj.obstacle.NumSurfaces
                    tmp_P_int = obj.FxyzGuv(OB,OA,obj.obstacle.SurfaceCoeffs{surf_id},obj.obstacle.SurfaceDegree(surf_id));
                    
                    if ~isempty(tmp_P_int)
                        P_int = [P_int,tmp_P_int];
                        P_index = [P_index;repmat(surf_id,size(tmp_P_int,2),2)];tmp_P_int = [];
                    end
                    tmp_P_int = obj.FxyzEt(OB,OA,obj.obstacle.SurfaceCoeffs{surf_id});
                    if ~isempty(tmp_P_int)
                        P_int = [P_int,tmp_P_int];
                        P_index = [P_index;repmat(surf_id,size(tmp_P_int,2),2)];tmp_P_int = [];
                    end
                end
                Gxyz_coeff = obj.GetGxyz(OB,OA);
                for bound_ind = 1:obj.obstacle.NumBoundary
                    if obj.obstacle.isTrigonometric(bound_ind) == 0
                    tmp_P_int = obj.GxyzFt(Gxyz_coeff,obj.obstacle.ParametricCoeffs{bound_ind},obj.obstacle.ParametricEqu{bound_ind});
                    else
                        tmp_P_int = obj.GxyzTrigonometric(Gxyz_coeff,obj.obstacle.ParametricEqu{bound_ind});
                    end
                    if ~isempty(tmp_P_int)
                        P_int = [P_int,tmp_P_int];
                        surf_id = obj.obstacle.P2SIndex(find(obj.obstacle.P2SIndex(:,1) == bound_ind),2:3);
                        P_index = [P_index;repmat(surf_id,size(tmp_P_int,2),1)];tmp_P_int = [];
                    end
                end
                if ~isempty(P_int)
                    [OA_u, P_int, P_surf_index] = obj.sortIntersectedPoints(OB,OA,P_int,P_index,obj.obstacle.ImplicitEqu,obj.obstacle.SurfacesDirection,obj.obstacle.ObstacleHull);
                end
                if ~isempty(OA_u)
                    OA_u = [OA(:,2),OA_u,OA(:,1)];
                    OA_u = unique(OA_u','rows','stable')';
                    feasible_interval_index = verifyInterval(obj,OB,OA_u,P_surf_index,obj.obstacle.ObstacleHull);
                    count = 1;
                    for int_id = 1:size(OA_u,2) - 1                       
                        if feasible_interval_index(int_id) == 1
                            feasibleInterval{cable_id}{count} = [OA_u(:,int_id), OA_u(:,int_id + 1)];
                            count = count + 1;
                        end
                    end
                else
                    feasibleInterval{cable_id}{1} = [reconfig_element.OA_begin(:,cable_id), reconfig_element.OA_end(:,cable_id)];
                end
                 else
                    feasibleInterval{cable_id}{1} = [obj.model.cableModel.r_OAs(1:3,cable_id),obj.model.cableModel.r_OAs(1:3,cable_id)];
                end
            end
            feasibleInterval = feasibleInterval';           
            
        end
        
        function feasible_interval = verifyInterval(obj,OB,OA_u,OA_surf_ind,obstacle_hull)
            surf_ind = unique(OA_surf_ind); P_intersected = [];
            for j = 1:size(OA_u,2) - 1
                P_inside = [];
                OA_mid = (OA_u(:,j) + OA_u(:,j+1))/2;
                E = [OB,OA_mid];
                for i = 1:size(surf_ind,1)
                    F = F_G_t_coeffs(E,obj.obstacle.SurfaceCoeffs{surf_ind(i)});
                    t = round(roots(F),8);
                    t = unique(t(imag(t) == 0));
%                     t(t>1) = [];
%                     t(t<0) = [];
                    if ~isempty(t)
                        P_intersected = (E(:,1) - E(:,2)) .* t' + E(:,2);
                        if inhull(P_intersected(:,i)',obstacle_hull.Vertices)
                            P_inside(i) = true;
                        end
%                         for kk = 1:size(P_intersected,2)

%                             for k = 1:obj.obstacle.NumSurfaces
%                                 P_side(k) = sign(round(obj.obstacle.ImplicitEqu{k}(P_intersected(1,kk),P_intersected(2,kk),P_intersected(3,kk)),3));
%                                 if P_side(k) == 0
%                                     P_side(k) = obj.obstacle.SurfacesDirection(k);
%                                 end
%                             end
%                             if all(P_side == obj.obstacle.SurfacesDirection)
%                                 P_inside(i,kk) = true;
%                             else
%                                 P_inside(i,kk) = false;
%                             end
%                         end
                    else
                        P_inside = false;
                    end
                    
                end
%                                 PJ = [OB,OA_u(:,j+1)]
%                                 line(PJ(1,:),PJ(2,:),PJ(3,:))
                
                P_inside = reshape(P_inside,1,[]);
                if any(P_inside)
                    feasible_interval(j) = false;
                else
                    feasible_interval(j) = true;
                end
            end
        end
    end
    methods (Static)
        function P_intersected = FxyzGuv(OB,OA,surfaceCoeffs,surfaceDegree)
            %% FxyzGuv
            P_intersected = [];
            if surfaceDegree == 4
                H0 = F_uv_H0_coeffs(OB,OA,surfaceCoeffs);
                H1 = F_uv_H1_coeffs(OB,OA,surfaceCoeffs);
                H2 = F_uv_H2_coeffs(OB,OA,surfaceCoeffs);
                H3 = F_uv_H3_coeffs(OB,OA,surfaceCoeffs);
                H4 = F_uv_H4_coeffs(OB,OA,surfaceCoeffs);
                discrminant = Discriminant_deg_4(H4,H3,H2,H1,H0);
            elseif surfaceDegree == 3
                H0 = F_uv_H0_coeffs(OB,OA,surfaceCoeffs);
                H1 = F_uv_H1_coeffs(OB,OA,surfaceCoeffs);
                H2 = F_uv_H2_coeffs(OB,OA,surfaceCoeffs);
                H3 = F_uv_H3_coeffs(OB,OA,surfaceCoeffs);
                discrminant = Discriminant_deg_3(H3,H2,H1,H0);
            elseif surfaceDegree == 2
                H0 = F_uv_H0_coeffs(OB,OA,surfaceCoeffs);
                H1 = F_uv_H1_coeffs(OB,OA,surfaceCoeffs);
                H2 = F_uv_H2_coeffs(OB,OA,surfaceCoeffs);
                discrminant = Discriminant_deg_2(H2,H1,H0);
            else
                return
            end
            
            u = round(roots(discrminant),9);
            u = unique(u(imag(u) == 0));
            u(u>1) = [];
            u(u<0) = [];
            if ~isempty(u)
                for i = 1:size(u,1)
                    v_coeffs = F_uv_v_coeffs(OB,OA,surfaceCoeffs,u(i));
                    v = roots(v_coeffs);
                    tmp_v = round(v,3);
                    [tmp_v,index] = unique(tmp_v(imag(tmp_v) == 0));
                    v = v(index(find(tmp_v<=1 & tmp_v>=0)));
                    v = real(v);
                    %                     v(v>1) = [];
                    %                     v(v<0) = [];
                    
                    %                     v = round(roots(v_coeffs),5);
                    %                     v = unique(v(imag(v) == 0));
                    
                    if ~isempty(v)
                        for j = 1:size(v,1)
                            P_intersected = [P_intersected,...
                                (((OA(:,1) - OA(:,2))*u(i) +  OA(:,2)) - OB) * v(j) + OB];
                        end
                        %                     LOA =@(u) (OA(:,1) - OA(:,2)).*u + OA(:,2);
                        %                     Guv_fun = @(u,v) OB + (LOA(u) - OB).*v;
                        %                     P_j = Guv_fun(u(1),v_roots(1))
                        %                     scatter3(P_j(1),P_j(2),P_j(3),'filled')
                        
                    end
                end
            end
        end
        
        function P_intersected = FxyzEt(OB,OA,surfaceCoeffs)
            P_intersected = [];
            E{1} = [OB,OA(:,1)];
            E{2} = [OB,OA(:,2)];
            E{3} = OA;
%             line(E{3}(1,:),E{3}(2,:),E{3}(3,:));
%             line(E{2}(1,:),E{2}(2,:),E{2}(3,:));
%             line(E{1}(1,:),E{1}(2,:),E{1}(3,:));
%             
            for i = 1:3
                F_G_t =  F_G_t_coeffs(E{i},surfaceCoeffs);
                t = round(roots(F_G_t),9);
                t = unique(t(imag(t) == 0));
                t(t>1) = [];
                t(t<0) = [];
                if ~isempty(t)
                    P_intersected = [P_intersected,(E{i}(:,1) - E{i}(:,2)) .* t' + E{i}(:,2)];
                end
            end
            
            
            
        end
        
        function P_intersected = GxyzTrigonometric(Gxyz,boundaryEqu)
            syms t
            P_intersected = [];
            X = [boundaryEqu(t);1];
            xsol = round(double(solve(Gxyz*X == 0,t)),9);
            for i = 1:numel(xsol)
            if isreal(xsol(i))
                P_intersected = [P_intersected,boundaryEqu(xsol(i))];                
            end
            end
        end
        
        function P_intersected = GxyzFt(Gxyz,boundaryEquCoeff,boundaryEqu)
            P_intersected = [];
            G_F_t = G_F_t_coeffs(Gxyz,boundaryEquCoeff);
            t = round(roots(G_F_t),9);
            t = unique(t(imag(t) == 0));
            t(t>1) = [];
            t(t<0) = [];
            if ~isempty(t)
                for i = 1:max(size(t))
                    P_intersected = [P_intersected,boundaryEqu(t(i))];
                end
            end
        end
        
        function Gxyz_coeff = GetGxyz(OB,OA)
            %fit Gxyz plane equation
            surface_data = [OB,OA]';
            vec_1 = surface_data(2,:) - surface_data(1,:);
            vec_2 = surface_data(end,:) - surface_data(1,:);
            v(1) = vec_1(2)*vec_2(3) - vec_1(3)*vec_2(2);
            v(2) = vec_1(3)*vec_2(1) - vec_1(1)*vec_2(3);
            v(3) = vec_1(1)*vec_2(2) - vec_1(2)*vec_2(1);
            v(4) = -v(1:3)*surface_data(1,:)';
            Gxyz_coeff = v;
            %             Gxyz = @(x,y,z) v*[x;y;z;1];
        end
        
        function [OA_u, P_out, P_surf_index] = sortIntersectedPoints(OB,OA,P_intersected,P_index,surf_equ, surf_dir,obstacle_hull)
            [P_intersected,ind] = unique(P_intersected','rows','stable');
            P_index = P_index(ind,:); P_intersected = P_intersected';
            OA_u_tmp = []; P_out = []; u_compare = []; OA_u =[]; P_surf_index = []; u = [];v = [];
            for i = 1:size(P_intersected,2)
                [tmp_u,tmp_v] =  G_uv(OA,OB,P_intersected(:,i));
                P_side = [];
                
                if ~isempty(tmp_u)
                    if inhull(P_intersected(:,i)',obstacle_hull.Vertices)
%                         u_compare = [u_compare,tmp_u];
                        P_surf_index = [P_surf_index;P_index(i,:)];
                        OA_u_tmp = [OA_u_tmp,(OA(:,1) - OA(:,2)) * tmp_u + OA(:,2)];
                        P_out = [P_out, P_intersected(:,i)];
                        u = [u,tmp_u];v = [v,tmp_v];
                    end
                    %%%%%%% need to deal with this, point inside where
%                     for j = 1:size(surf_equ,2)
%                         P_side(j) = sign(round(surf_equ{j}(P_intersected(1,i),P_intersected(2,i),P_intersected(3,i)),3));
%                         if P_side(j) == 0
%                             P_side(j) = surf_dir(j);
%                         end
%                     end
%                     
%                     if all(P_side == surf_dir)                    
%                         u_compare = [u_compare,tmp_u];
%                         P_surf_index = [P_surf_index;P_index(i,:)];
%                         OA_u_tmp = [OA_u_tmp,(OA(:,1) - OA(:,2)) * tmp_u + OA(:,2)];
%                         P_out = [P_out, P_intersected(:,i)];
%                         u = [u,tmp_u];v = [v,tmp_v];
%                     end
                    
                end
            end

            [u,index_order] = unique(u','rows','sorted');
            OA_u = OA_u_tmp(:,index_order);
            
            P_surf_index = P_surf_index(index_order,:);
        end
        
        
    end
end