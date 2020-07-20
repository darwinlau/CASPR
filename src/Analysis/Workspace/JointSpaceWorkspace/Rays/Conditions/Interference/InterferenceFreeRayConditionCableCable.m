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
        function w = InterferenceFreeRayConditionCableCable(model, min_ray_lengths, magin_dofs)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.dofMargins = magin_dofs;
        end
            
        % Evaluate the interference free intervals 
        function intervals =  evaluateFunction(obj, model, ws_ray)
            interference_q = [];
            % Variable initialisation
            q_zero = zeros(obj.numDofs, 1);            
            free_variable_index = ws_ray.free_variable_index;
            is_dof_translation = obj.areDofsTranslation(free_variable_index);
            
            if is_dof_translation
                maximum_degree = obj.MAX_DEGREE_TRANSLATION;
            else
                maximum_degree = obj.MAX_DEGREE_ORIENTATION;
            end
            
            % ASSUME THAT EVERY CABLE IS ONE SEGMENT FOR NOW
            cable_combinations = nchoosek(1:obj.numCables, 2);
            num_cable_combs = size(cable_combinations, 1);
            
            free_var_lin_space_q = ws_ray.free_variable_range(1):(ws_ray.free_variable_range(2)-ws_ray.free_variable_range(1))/maximum_degree:ws_ray.free_variable_range(2);
            
            if is_dof_translation
                free_var_lin_space_u = free_var_lin_space_q;
            else
                free_var_lin_space_u = tan(free_var_lin_space_q/2);
            end
            
            % Pose data
            q_fixed = ws_ray.fixed_variables;
            fixed_index = true(obj.numDofs,1); 
            fixed_index(ws_ray.free_variable_index) = false;
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
                    if (isreal(root_i_u) && (root_i_q >= ws_ray.free_variable_range(1)) && (root_i_q <= ws_ray.free_variable_range(2)))
            
                        q(free_variable_index) = root_i_q;
                        
                        model.update(q, q_zero, q_zero, q_zero);
                        i = cable_combinations(k, 1);
                        j = cable_combinations(k, 2);
                        A_i = model.cableModel.r_OAs(1:3, i);
                        B_i = model.cableModel.r_OAs(4:6, i);
                        A_j = model.cableModel.r_OAs(1:3, j);
                        B_j = model.cableModel.r_OAs(4:6, j);
                        [t_i, t_j] = obj.intersection_xyz(A_i, B_i, A_j, B_j);
                        
                        if (t_i >= 0 && t_i <= 1 && t_j >= 0 && t_j <= 1)
                            interference_q = [interference_q; q(free_variable_index)];
                            cable_combinations(k, :)
%                             g
                            q
%                             A_i
%                             B_i 
%                             A_j
%                             B_j
                            metric = MinCableCableDistanceMetric();
                            metric.evaluate(model)
                        end
                    end
                end
            end
            g_coeffs
            
            intervals = [];
            
%             intervals = obj.evaluate_intersection(model, workspace_ray, maximum_degree, free_variable_index, free_variable_linear_space, least_squares_matrix_i);
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
            
            A_adj = adjoint(A);
            x = A_adj*b;
            n1 = x(1);
            n2 = x(2);
            d = det(A);
            
            ti = n1/d;
            tj = n2/d;
            g = n1*(B_i(3)-A_i(3)) - n2*(B_j(3)-A_j(3)) - d*(A_j(3)-A_i(3));
        end
    end
end

