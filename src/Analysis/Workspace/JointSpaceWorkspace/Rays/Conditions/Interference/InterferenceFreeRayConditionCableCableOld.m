% Class to compute whether a pose (dynamics) is within the interference
% free workspace (IFW)
%
% Author        : Darwin Lau
% Created       : 2020
% Description   :

classdef InterferenceFreeRayConditionCableCableOld < WorkspaceRayConditionBase
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
        dofMargins;
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCableCableOld(model, min_ray_lengths, dof_margins)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.dofMargins = dof_margins;
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            % Variable initialisation
            q_zero = zeros(obj.numDofs, 1);
            free_variable_index = ws_ray.freeVariableIndex;
            free_variable_lower = ws_ray.freeVariableRange(1);
            free_variable_upper = ws_ray.freeVariableRange(2);
            
            is_compiled_mode = (model.modelMode == ModelModeType.COMPILED);
            
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            dof_margin = obj.dofMargins(free_variable_index);
            
            if is_dof_translation
                maximum_degree = obj.MAX_DEGREE_TRANSLATION;
            else
                maximum_degree = obj.MAX_DEGREE_ORIENTATION;
            end
            
            % Go through every cables and segments
            cable_combinations = nchoosek(1:size(model.cableModel.r_OAs, 2), 2);
            num_cable_combs = size(cable_combinations, 1);
            
            free_var_lin_space_q = free_variable_lower:(free_variable_upper-free_variable_lower)/maximum_degree:free_variable_upper;
            
            if is_dof_translation
                free_var_lin_space_u = free_var_lin_space_q;
            else
                free_var_lin_space_u = tan(free_var_lin_space_q/2);
            end
            
            % Pose data
            q_fixed = ws_ray.fixedVariables;
            fixed_index = true(obj.numDofs,1);
            fixed_index(ws_ray.freeVariableIndex) = false;
            q = q_zero;
            q(fixed_index) = q_fixed;
            
            g_samples = zeros(num_cable_combs, maximum_degree+1);
            g_coeffs = zeros(num_cable_combs, maximum_degree+1);
            
            for linear_space_index = 1:maximum_degree+1
                % Update the value for q
                q_free = free_var_lin_space_q(linear_space_index);
                q(free_variable_index) = q_free;
                
                % Update the model
                if ~is_compiled_mode
                    model.update(q, q_zero, q_zero, q_zero);
                    r_OAs = model.cableModel.r_OAs;
                else
                    r_OAs = model.cableModel.compiled_r_OAs_fn(q, q_zero, q_zero, q_zero);
                end
                
                for k = 1:num_cable_combs
                    i = cable_combinations(k, 1);
                    j = cable_combinations(k, 2);
                    A_i = r_OAs(1:3, i);
                    B_i = r_OAs(4:6, i);
                    A_j = r_OAs(1:3, j);
                    B_j = r_OAs(4:6, j);
                    g_samples(k, linear_space_index) = intersection_xyz(obj,A_i, B_i, A_j, B_j);
                    if ~is_dof_translation
                        g_samples(k, linear_space_index) = (1 + (tan(q_free/2))^2)*g_samples(k, linear_space_index);
                    end
                end
            end
            if is_dof_translation  
                intersect_roots_u = ws_ray.freeVariableRange';
            else
                intersect_roots_u = tan(ws_ray.freeVariableRange'/2);
            end
            intervals_count = 1;intervals = [];
            for k = 1:size(g_samples,1)
           
                g_coeffs(k, :) = GeneralMathOperations.PolynomialFit(free_var_lin_space_u', g_samples(k, :)', maximum_degree);
                g_coeffs(k, :) = round(g_coeffs(k, :), obj.ROUNDING_DIGIT);
                intersect_roots_u = [intersect_roots_u;roots(g_coeffs(k, :))];

            end
            %g_coeffs
            intersect_roots_u = intersect_roots_u(imag(intersect_roots_u) == 0);
            intersect_roots_u = unique(intersect_roots_u);
            previous_intersected = 0;
            for r = 1:size(intersect_roots_u,1)
                    root_i_u = intersect_roots_u(r);
                    if is_dof_translation
                        root_i_q = root_i_u;
                    else
                        root_i_q = 2*atan(root_i_u);
                    end
                    if (isreal(root_i_u) && (root_i_q >= free_variable_lower) && (root_i_q <= free_variable_upper))
                        
                        q(free_variable_index) = root_i_q;
                        
                        % Update the model
                        if ~is_compiled_mode
                            model.update(q, q_zero, q_zero, q_zero);
                            r_OAs = model.cableModel.r_OAs;
                        else
                            r_OAs = model.cableModel.compiled_r_OAs_fn(q, q_zero, q_zero, q_zero);
                        end
                        for k = 1:num_cable_combs
                            i = cable_combinations(k, 1);
                            j = cable_combinations(k, 2);
                            A_i = r_OAs(1:3, i);
                            B_i = r_OAs(4:6, i);
                            A_j = r_OAs(1:3, j);
                            B_j = r_OAs(4:6, j);
                            [t_i(k,:), t_j(k,:)] = intersection_titj(obj,A_i, B_i, A_j, B_j);
                        end
                        if sum((t_i>=0 & t_i<=1) & (t_j>=0 & t_j<=1)) > 0 % exist intersection
%                             model.update(q, q_zero, q_zero, q_zero);
%                             metric = MinCableCableDistanceMetric();
%                             metric.evaluate(model)
                            
                            previous_intersected = 1;
                            if isempty(intervals)
                                intervals(1,1) = q(free_variable_index);    
                            else                            
                                intervals(intervals_count,2) = q(free_variable_index);                                
                                intervals_count = intervals_count + 1;
                            end
                        else %no intersection
                            if previous_intersected == 0
                                if isempty(intervals)
                                    intervals(1,1) = q(free_variable_index);
                                else
                                    intervals(end,2) =  q(free_variable_index);
                                end
                            else                                
                                intervals(intervals_count,1:2) = [intervals(end),q(free_variable_index)];
                            end
                            previous_intersected = 0;
                        end
                    end
                
            end
            
            if size(intervals,1)>1 
                interval_bound = intervals([1,end]);
                intervals(:,1) =  intervals(:,1) + dof_margin/2;
                intervals(:,2) =  intervals(:,2) - dof_margin/2;
                intervals([1,end]) = interval_bound;
                
            end
            
        end
        
    end
    
    methods (Access = private)
        function g = intersection_xyz(obj, A_i, B_i, A_j, B_j)

            A = [B_i(1)-A_i(1), -B_j(1)+A_j(1); B_i(2)-A_i(2), -B_j(2)+A_j(2)];
            b = [A_j(1)-A_i(1); A_j(2)-A_i(2)];
            % Adjoint of A
            A_adj = [A(2,2) -A(1,2); -A(2,1) A(1,1)];
            x = A_adj*b;
            n1 = x(1);
            n2 = x(2);
            % Determinant of A
            d = A(1,1)*A(2,2) - A(1,2)*A(2,1);            
            g = n1*(B_i(3)-A_i(3)) - n2*(B_j(3)-A_j(3)) - d*(A_j(3)-A_i(3));

%              g = cross((B_i-A_i),(B_j-A_j))'*(A_i-A_j);
        end
        function [ti,tj] = intersection_titj(obj, A_i, B_i, A_j, B_j)
            a = B_i-A_i;
            b = B_j-A_j;
            c = [a(2)*b(3) - a(3)*b(2) ; a(3)*b(1) - a(1)*b(3) ; a(1)*b(2) - a(2)*b(1)];
            d = c(1) * (A_i(1)-A_j(1)) + c(2) * (A_i(2)-A_j(2)) + c(3) * (A_i(3)-A_j(3));
            if round(d,obj.ROUNDING_DIGIT) == 0
            b = [A_j(1)-A_i(1); A_j(2)-A_i(2)];
            A = [B_i(1)-A_i(1), -B_j(1)+A_j(1); B_i(2)-A_i(2), -B_j(2)+A_j(2)];
            
            
            % Adjoint of A
            A_adj = [A(2,2) -A(1,2); -A(2,1) A(1,1)];
            x = A_adj*b;
            n1 = x(1);
            n2 = x(2);
            % Determinant of A
            d = A(1,1)*A(2,2) - A(1,2)*A(2,1); 
            
            ti = n1/d;
            tj = n2/d;
            if (ti == 0 && tj == 0 ) || (isnan(ti) && isnan(tj))
                b = [A_j(1)-A_i(1); A_j(3)-A_i(3)];
                A = [B_i(1)-A_i(1), -B_j(1)+A_j(1); B_i(3)-A_i(3), -B_j(3)+A_j(3)];
                
                
                % Adjoint of A
                A_adj = [A(2,2) -A(1,2); -A(2,1) A(1,1)];
                x = A_adj*b;
                n1 = x(1);
                n2 = x(2);
                % Determinant of A
                d = A(1,1)*A(2,2) - A(1,2)*A(2,1);
                ti = n1/d;
                tj = n2/d;
            end
            else
              ti = Inf;
              tj = Inf;
            end
        end
        
    end
end

