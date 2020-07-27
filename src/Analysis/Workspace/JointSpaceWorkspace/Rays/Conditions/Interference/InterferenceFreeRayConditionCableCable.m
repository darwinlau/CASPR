% Class to compute whether a pose (dynamics) is within the interference 
% free workspace (IFW)
%
% Author        : Darwin Lau
% Created       : 2020
% Description   : 

classdef InterferenceFreeRayConditionCableCable < WorkspaceRayConditionBase
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
        function w = InterferenceFreeRayConditionCableCable(model, min_ray_lengths, dof_margins)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.dofMargins = dof_margins;
        end
            
        % Evaluate the interference free intervals 
        function intervals =  evaluateFunction(obj, model, ws_ray)
            interference_q = [];
            % Variable initialisation
            q_zero = zeros(obj.numDofs, 1);            
            free_variable_index = ws_ray.freeVariableIndex;
            free_variable_lower = ws_ray.freeVariableRange(1);
            free_variable_upper = ws_ray.freeVariableRange(2);
            
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
                model.update(q, q_zero, q_zero, q_zero);
                
                for k = 1:num_cable_combs
                    i = cable_combinations(k, 1);
                    j = cable_combinations(k, 2);
                    A_i = model.cableModel.r_OAs(1:3, i);
                    B_i = model.cableModel.r_OAs(4:6, i);
                    A_j = model.cableModel.r_OAs(1:3, j);
                    B_j = model.cableModel.r_OAs(4:6, j);
                    [~, ~, g_samples(k, linear_space_index)] = obj.intersection_xyz(A_i, B_i, A_j, B_j);
                    if ~is_dof_translation
                        g_samples(k, linear_space_index) = (1 + (tan(q_free/2))^2)*g_samples(k, linear_space_index);
                    end
                end
            end
            
            
            for k = 1:num_cable_combs
                g_coeffs(k, :) = GeneralMathOperations.PolynomialFit(free_var_lin_space_u', g_samples(k, :)', maximum_degree);
                g_coeffs(k, :) = round(g_coeffs(k, :), obj.ROUNDING_DIGIT); 
                intersect_roots_u = roots(g_coeffs(k, :));
                for r = 1:length(intersect_roots_u)
                    root_i_u = intersect_roots_u(r);
                    if is_dof_translation
                        root_i_q = root_i_u;
                    else
                        root_i_q = 2*atan(root_i_u);
                    end
                    if (isreal(root_i_u) && (root_i_q >= free_variable_lower) && (root_i_q <= free_variable_upper))
            
                        q(free_variable_index) = root_i_q;
                        
                        model.update(q, q_zero, q_zero, q_zero);
                        i = cable_combinations(k, 1);
                        j = cable_combinations(k, 2);
                        A_i = model.cableModel.r_OAs(1:3, i);
                        B_i = model.cableModel.r_OAs(4:6, i);
                        A_j = model.cableModel.r_OAs(1:3, j);
                        B_j = model.cableModel.r_OAs(4:6, j);
                        [t_i, t_j] = obj.intersection_xyz(A_i, B_i, A_j, B_j);
                        
                        if (t_i > 0 && t_i < 1 && t_j > 0 && t_j < 1)
                            interference_q = [interference_q; q(free_variable_index)];
%                             cable_combinations(k, :)
%                             q(free_variable_index)
%                             metric = MinCableCableDistanceMetric();
%                             metric.evaluate(model)
                        end
                    end
                end
            end
            
            intervals_count = 0;
            if ~isempty(interference_q)
                interference_q = sort(interference_q);
                intervals = zeros(length(interference_q)+1, 2);
                
                if (free_variable_lower < interference_q(1)-dof_margin/2) 
                    intervals_count = intervals_count + 1;
                    intervals(intervals_count,:) = [free_variable_lower, interference_q(1)-dof_margin/2];
                end
                
                for i = 2:length(interference_q)
                    if (interference_q(i-1)+dof_margin/2 < interference_q(i)-dof_margin/2)
                        intervals_count = intervals_count + 1;
                        intervals(intervals_count,:) = [interference_q(i-1)+dof_margin/2, interference_q(i)-dof_margin/2];
                    end
                end
                
                if (interference_q(end)+dof_margin/2 < free_variable_upper) 
                    intervals_count = intervals_count + 1;
                    intervals(end, :) = [interference_q(end)+dof_margin/2, free_variable_upper];
                end                
                intervals = intervals(1:intervals_count, :);
            else
                intervals = [free_variable_lower free_variable_upper];
            end
            
            
            
%             interference_q
%             intervals
            
%             
%             % Run though and ensure that the identified intervals are
%             % larger than the tolerance
%             count = 1;
%             for iteration_index = 1:size(intervals,1)
%                 if(intervals(count,2) - intervals(count,1) < obj.minRayLengths(free_variable_index))
%                     intervals(count,:) = [];
%                 else
%                     count = count+1;
%                 end
%             end
        end
        
    end
    
    methods (Access = private)
        function [ti, tj, g] = intersection_xyz(~, A_i, B_i, A_j, B_j)
            A = [B_i(1)-A_i(1), -B_j(1)+A_j(1); B_i(2)-A_i(2), -B_j(2)+A_j(2)];
            b = [A_j(1)-A_i(1); A_j(2)-A_i(2)];
            
            % Adjoint of A
            A_adj = [A(2,2) -A(1,2); -A(2,1) A(1,1)];
            x = A_adj*b;
            n1 = x(1);
            n2 = x(2);
            % Determinant of A
            d = A(1,1)*A(2,2) - A(1,2)*A(2,1);
            
            ti = n1/d;
            tj = n2/d;
            g = n1*(B_i(3)-A_i(3)) - n2*(B_j(3)-A_j(3)) - d*(A_j(3)-A_i(3));
        end
    end
end

