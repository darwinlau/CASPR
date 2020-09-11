% Class to compute whether a pose (dynamics) is within the interference
% free workspace (IFW)
%
% Author        : Paul Cheng
% Created       : 2020
% Description   :

classdef InterferenceFreeRayConditionCableQuadraticObstacle < WorkspaceRayConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 9;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceRayConditionType.INTERFERENCE_CABLE_QUADSURF;
        MAX_DEGREE_TRANSLATION = 1;
        MAX_DEGREE_ORIENTATION = 2;
    end
    
    properties (SetAccess = protected)
        % Set constants
        areDofsTranslation;         % Array for the q of the joint (true if translation and false if rotation)
        numDofs;                    % The number of dofs
        numCables;                  % The number of cables
        QuadSurf;                   % The obstacle surface equations
        surface_bound;               % Obstacle boundary
        numSurf
        is_compiled_mode;
        segment_num;
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCableQuadraticObstacle(model, min_ray_lengths, QuadSurf)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.QuadSurf = QuadSurf.Quad_Matrix;
            w.surface_bound = QuadSurf.boundary;
            w.is_compiled_mode = (model.modelMode == ModelModeType.COMPILED);
            w.numSurf = size(QuadSurf.Quad_Matrix,2);
            w.segment_num = size(model.cableModel.r_OAs,2);
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
                        
            [OA_i_begin,OB_i_begin] = obj.GetSegmentData(model,q_begin);
            [OA_i_end,OB_i_end] = obj.GetSegmentData(model,q_end);
            for i = 1:size(OA_i_begin,1)
                [OA_i_hat(i,:),~] = obj.LineIntersection3D([OB_i_begin(i,:);OB_i_end(i,:)],[OA_i_begin(i,:);OA_i_end(i,:)]);
            end
            
            for surf_ind = 1:size(obj.QuadSurf,2)
                QuadSurfCoeff(surf_ind,:)  = obj.GetQuadSurfCoeff(obj.QuadSurf{surf_ind});
            end
            u_value = [];
            if is_dof_translation
                for surf_ind = 1:size(obj.QuadSurf,2)
                [~,u_surface,segment_ind_surf] = obj.SurfaceIntersection(model,q_begin,q_end,surf_ind,free_variable_index,OA_i_hat);
                
                [~,u_segment,segment_ind_surf] = obj.SegmentIntersection(OB_i_begin,OB_i_end,surf_ind);
                
                u_value = [u_value;u_surface;u_segment];
                end
                q_intersected = (q_end - q_begin)*u_value' + q_begin;
            else
                %% find surface surface/edge intersection
                for surf_ind = 1:size(obj.QuadSurf,2)
                [~,u_surface,segment_ind_surf] = obj.SurfaceIntersection(model,q_begin,q_end,surf_ind,free_variable_index,OA_i_hat);
                
                [~,u_curve,segment_ind_curv] = obj.CurveIntersection(model,q_begin,q_end,surf_ind,free_variable_index);
                
                u_value = [u_value;u_surface;u_curve];
                end
                q_intersected = repmat(q_begin,1,size(u_value,1));
                q_intersected(free_variable_index,:) = 2*atan(u_value);
            end
            
            
            q_intersected = [q_intersected,q_begin,q_end];
            %% verify the intersection interval
            q_intersected = unique(round(q_intersected',obj.ROUNDING_DIGIT),'rows')';
            
            for i = 1:size(q_intersected,2) - 1
                
                has_intersected = obj.IntervalVerify(model,QuadSurfCoeff,q_intersected(:,i),q_intersected(:,i+1));
                
                if ~has_intersected
                    if ~isempty(intervals) && intervals(end) == q_intersected(free_variable_index,i)
                        intervals(end) = q_intersected(free_variable_index,i+1);
                    else
                        intervals(intervals_count,:) = [q_intersected(free_variable_index,i),q_intersected(free_variable_index,i+1)];
                        intervals_count = intervals_count + 1;
                    end
                    
                end
                
                
            end
            
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
        
        %function to find intersection between curve edge and quadratic surface
        function [flag,u_val,segment_ind] = SurfaceIntersection(obj,model,q_begin,q_end,surf_ind,free_variable_index,OA_i_hat)
            u_coeff = [];v_coeff = [];
            if ~obj.areDofsTranslation(free_variable_index)
                sample_size = 5;
                q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
                u_sample = tan(q_sample/2);
                H_1_u_denominator = (1+u_sample.^2).^2;
                H_2_u_denominator = (1+u_sample.^2);
                H_3_u_denominator = [1 1 1 1 1];
                H_1_deg = 4;
                H_2_deg = 2;
                H_3_deg = 0;
            else
                sample_size = 3;
                q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
                u_sample = linspace(0,1,sample_size);
                H_1_u_denominator = [1 1 1];
                H_2_u_denominator = [1 1 1];
                H_3_u_denominator = [1 1 1];
                H_1_deg = 2;
                H_2_deg = 1;
                H_3_deg = 0;
            end
            v = linspace(0,1,sample_size);
            segment_ind =  repmat(1:obj.segment_num,sample_size-1,1); segment_ind = segment_ind(:);
            
            
            q_update = q_begin;
            u_sample_per_pose = [];
            for i = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(i);
                [~,OB_i_u]  = obj.GetSegmentData(model,q_update);
                u_sample_per_cable = [];
                %                 G_uv_v(i,:) = reshape(((OB_i_u - OA_i_hat)*v_sample(i) + OA_i_hat),1,obj.segment_num*3);
                for j = 1:size(v,2)
                    G_uv_v{j} = ((OB_i_u - OA_i_hat)*v(j) + OA_i_hat);
                end
                %                 G_uv_v_0 = ((OB_i_u - OA_i_hat)*0 + OA_i_hat); % v =
                %                 G_uv_v_1 = ((OB_i_u - OA_i_hat)*0.25 + OA_i_hat); % v = 0.25
                %                 G_uv_v_2 = ((OB_i_u - OA_i_hat)*0.5 + OA_i_hat); % v = 0.5
                %                 G_uv_v_3 = ((OB_i_u - OA_i_hat)*0.75 + OA_i_hat); % v = 1
                %                 G_uv_v_4 = ((OB_i_u - OA_i_hat)*1 + OA_i_hat); % v = 1
                for j = 1:obj.segment_num
                    for k = 1:size(v,2)
                        v_sample(k) = [G_uv_v{k}(j,:) 1]*obj.QuadSurf{surf_ind}*[G_uv_v{k}(j,:) 1]';
                    end
                    %                     v_sample(1) = [G_uv_v_0(j,:) 1]*obj.QuadSurf{surf_ind}*[G_uv_v_0(j,:) 1]';
                    %                     v_sample(2) = [G_uv_v_1(j,:) 1]*obj.QuadSurf{surf_ind}*[G_uv_v_1(j,:) 1]';
                    %                     v_sample(3) = [G_uv_v_2(j,:) 1]*obj.QuadSurf{surf_ind}*[G_uv_v_2(j,:) 1]';
                    %                     v_sample(4) = [G_uv_v_3(j,:) 1]*obj.QuadSurf{surf_ind}*[G_uv_v_3(j,:) 1]';
                    %                     v_sample(5) = [G_uv_v_4(j,:) 1]*obj.QuadSurf{surf_ind}*[G_uv_v_4(j,:) 1]';
                    v_coeff{i}(:,j) = GeneralMathOperations.PolynomialFit(v', v_sample', 2);
                end
            end
            for j = 1:obj.segment_num
                for i = 1:size(v_coeff,2)
                    %                     H_1_u_sample(j,i) = v_coeff{i}(1,j) *(1+u_sample(i)^2)^2;
                    %                     H_2_u_sample(j,i) = v_coeff{i}(2,j) *(1+u_sample(i)^2);
                    %                     H_3_u_sample(j,i) = v_coeff{i}(3,j);
                    H_1_u_sample(j,i) = v_coeff{i}(1,j) * H_1_u_denominator(i);
                    H_2_u_sample(j,i) = v_coeff{i}(2,j) * H_2_u_denominator(i);
                    H_3_u_sample(j,i) = v_coeff{i}(3,j) * H_3_u_denominator(i);
                end
                H_1_coeff = GeneralMathOperations.PolynomialFit(u_sample', H_1_u_sample(j,:)', H_1_deg)';
                H_2_coeff = GeneralMathOperations.PolynomialFit(u_sample', H_2_u_sample(j,:)', H_2_deg)';
                H_3_coeff = GeneralMathOperations.PolynomialFit(u_sample', H_3_u_sample(j,:)', H_3_deg)';
                
                u_coeff(j,:) = obj.SumCoeff({conv(H_2_coeff,H_2_coeff);-4*H_1_coeff*H_3_coeff(end)});
            end
            
            u_val = [];
            for i = 1:obj.segment_num
                u_roots = roots(u_coeff(i,:));
                u_val = [u_val;u_roots];
                
            end
            
            real_sol_index = find(imag(u_val) == 0);
            u_val = u_val(imag(u_val)==0);
            [u_val,ia,~] = unique(u_val);
            real_sol_index = real_sol_index(ia,:);
            remove_index = [find(u_val < u_sample(1));find(u_val > u_sample(end))];
            real_sol_index(remove_index) = [];
            u_val(remove_index) = [];
            segment_ind = segment_ind(real_sol_index);
            if ~isempty(u_val)
                flag = 1;
            else
                flag = 0;
            end
            
        end
        
        %function to find intersection between curve edge and quadratic surface
        function [flag,u_val,segment_ind] = CurveIntersection(obj,model,q_begin,q_end,surf_ind,free_variable_index)
            %             OB_i_u = GetOBiu(obj,model,q_begin,q_end,free_variable_index);
            %             S = obj.GetQuadSurfCoeff(obj.QuadSurf{surf_ind})
            %             u_c = [1 0 1];
            %             for i = 1:obj.segment_num
            %                 u_x = OB_i_u{i}(1,:);u_y = OB_i_u{i}(2,:);u_z = OB_i_u{i}(3,:);
            %                 u_coeff1(i,:) =  SumCoeff({S(1)*conv(u_x,u_x);S(2)*conv(u_y,u_y);S(3)*conv(u_z,u_z);...
            %                     S(4)*conv(u_x,u_y);S(5)*conv(u_x,u_z);S(6)*conv(u_y,u_z);...
            %                     S(7)*conv(u_x,u_c);S(8)*conv(u_y,u_c); S(9)*conv(u_z,u_c);S(10)*conv(u_c,u_c)});
            %             end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            u_val = [];
            sample_size = 5;
            segment_ind =  repmat(1:obj.segment_num,sample_size-1,1); segment_ind = segment_ind(:);
            
            q_sample = linspace(q_begin(free_variable_index),q_end(free_variable_index),sample_size);
            u_sample = tan(q_sample/2);
            q_update = q_begin;
            for i = 1:size(q_sample,2)
                q_update(free_variable_index) = q_sample(i);
                [~,OB_i_u]  = obj.GetSegmentData(model,q_update);
                for j = 1:obj.segment_num
                    f_sample(i,j) = ([OB_i_u(j,:) 1]*obj.QuadSurf{surf_ind}*[OB_i_u(j,:) 1]')*(1+u_sample(i)^2)^2;
                end
            end
            for i = 1:obj.segment_num
                u_coeff(i,:) = GeneralMathOperations.PolynomialFit(u_sample', f_sample(:,i), 4)';
                u_val = [u_val;roots(u_coeff(i,:))];
            end
            real_sol_index = find(imag(u_val) == 0);
            u_val = u_val(imag(u_val)==0);
            [u_val,ia,~] = unique(u_val);
            real_sol_index = real_sol_index(ia,:);
            remove_index = [find(u_val < u_sample(1));find(u_val > u_sample(end))];
            real_sol_index(remove_index) = [];
            u_val(remove_index) = [];
            segment_ind = segment_ind(real_sol_index);
            
            if ~isempty(u_val)
                flag = 1;
            else
                flag = 0;
            end
        end
        
        %function to find intersection between linear edge and quadratic surface
        function [flag,v_val,segment_ind] = SegmentIntersection(obj,P_begin,P_end,surf_ind)
            
            sample_size = 3;
            segment_ind =  repmat(1:size(P_begin,1),sample_size-1,1); segment_ind = segment_ind(:);
            
            v_sample = linspace(0,1,sample_size);
            
            v_val = [];
            
            for i = 1:size(v_sample,2)
                Si_t = ((P_end - P_begin)*v_sample(i) + P_begin);
                for j = 1:obj.segment_num
                    f_sample(i,j) = [Si_t(j,:) 1]*obj.QuadSurf{surf_ind}*[Si_t(j,:) 1]';
                    %                             v_coeff{i}(:,j) = GeneralMathOperations.PolynomialFit(linspace(0,1,2*obj.MAX_DEGREE_ORIENTATION+1)', v_sample', 2);
                end
            end
            for i = 1:obj.segment_num
                t_coeff(i,:) = GeneralMathOperations.PolynomialFit(v_sample', f_sample(:,i), 2)';
                v_val = [v_val;roots( t_coeff(i,:))];
            end
            %                 end
            
            real_sol_index = find(imag(v_val) == 0);
            %             u_value = unique((roots(u_coeff)));
            v_val = v_val(imag(v_val)==0);
            [v_val,ia,~] = unique(v_val);
            real_sol_index = real_sol_index(ia,:);
            %             u_value = u_value(imag(u_value)==0);
            remove_index = [find(v_val < 0);find(v_val > 1)];
            real_sol_index(remove_index) = [];
            v_val(remove_index) = [];
            segment_ind = unique(segment_ind(real_sol_index));
            if ~isempty(v_val)
                flag = 1;
            else
                flag = 0;
            end
        end
        
        %% function to check the intersected interval valid
        function has_intersected = IntervalVerify(obj,model,QuadSurfCoeff,q1,q2)
            [OA_i,OB_i] = obj.GetSegmentData(model,(q1+q2)/2);
            for surf_ind = 1:size(QuadSurfCoeff,1)
                [flag(surf_ind),~,~] = obj.SegmentIntersection(OB_i,OA_i,surf_ind);
            end
            if any(flag == 1)
                has_intersected = 1;
            else
                has_intersected = 0;
            end
        end
        
        %% function to get the coeffcient of the quadratic surface
        function Q  = GetQuadSurfCoeff(~,QuadSurf)
            %              Q = sym('s%d',[1 10]);
            Q = zeros(1,10);
            Q(1) = QuadSurf(1,1);
            Q(2) = QuadSurf(2,2);
            Q(3) = QuadSurf(3,3);
            Q(4) = 2*QuadSurf(1,2);
            Q(5) = 2*QuadSurf(2,3);
            Q(6) = 2*QuadSurf(1,3);
            Q(7) = 2*QuadSurf(1,4);
            Q(8) = 2*QuadSurf(2,4);
            Q(9) = 2*QuadSurf(3,4);
            Q(10) = QuadSurf(4,4);
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
        
        function output = SumCoeff(~,P)
            output = P{1,:};
            for i = 2:size(P,1)
                output =  [zeros(1, size(P{i,:},2)-size(output,2)) output] + [zeros(1, size(output,2)-size(P{i,:},2)) P{i,:}];
            end
        end
        
    end
end

