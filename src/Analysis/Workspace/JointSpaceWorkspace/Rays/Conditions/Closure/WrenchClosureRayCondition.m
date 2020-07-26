% Class to compute whether a pose (dynamics) is within the wrench-closure
% workspace (WCW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
classdef WrenchClosureRayCondition < WorkspaceRayConditionBase
    properties (Constant)
        % Fixed constants
        TOLERANCE = 1e-8;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = WorkspaceRayConditionType.WRENCH_CLOSURE;
    end 
    
    properties (SetAccess = protected)
        % Set constants
        areDofsTranslation;         % Array for the q of the joint (true if translation and false if rotation)
        numDofs;                    % The number of dofs
        numCables;                  % The number of cables
        degRedundancy;              % The degree of redundancy   
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchClosureRayCondition(model, min_ray_lengths)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            w.degRedundancy = w.numCables - w.numDofs;
        end
        
        % The taghirad inspired method
        % Intervals are of the form of 
        % [min_1 max_1]
        % [min_2 max_2] 
        % [min_3 max_3] 
        % [min_4 max_4] etc...
        function intervals = evaluateFunction(obj, model, workspace_ray)
            %% Variable initialisation
            free_variable_index = workspace_ray.freeVariableIndex;
            % Use the joint type to determine the maximum polynomial
            % degrees
            if(obj.areDofsTranslation(free_variable_index))
                % THIS MAY NEED TO BE CHANGED
                maximum_degree = obj.numDofs;
                % Set up a linear space for the free variable
                free_variable_linear_space = workspace_ray.freeVariableRange(1):(workspace_ray.freeVariableRange(2)-workspace_ray.freeVariableRange(1))/maximum_degree:workspace_ray.freeVariableRange(2);
                % Matrix for least squares computations
                least_squares_matrix = GeneralMathOperations.ComputeLeastSquareMatrix(free_variable_linear_space',maximum_degree);
            else
                % THIS MAY NEED TO BE CHANGED
                maximum_degree = 2*obj.numDofs+3; % temporarily added 3 to increase the accuracy
%                 maximum_degree = 2*obj.numDofs;
                % Set up a linear space for the free variable
                free_variable_linear_space = workspace_ray.freeVariableRange(1):(workspace_ray.freeVariableRange(2)-workspace_ray.freeVariableRange(1))/maximum_degree:workspace_ray.freeVariableRange(2);
                % Matrix for least squares computations
                least_squares_matrix = GeneralMathOperations.ComputeLeastSquareMatrix(tan(0.5*free_variable_linear_space)',maximum_degree);
            end
            % Take the inverse of the least squares matrix
            least_squares_matrix_i = inv(least_squares_matrix);
            if(obj.degRedundancy == 1)
                intervals = obj.evaluate_function_fully_restrained(model,workspace_ray,maximum_degree,free_variable_index,free_variable_linear_space,least_squares_matrix_i);
            else
                intervals = obj.evaluate_function_redundantly_restrained(model,workspace_ray,maximum_degree,free_variable_index,free_variable_linear_space,least_squares_matrix_i);
            end
            
            % Run though and ensure that the identified intervals are
            % larger than the tolerance
            count = 1;
            for iteration_index = 1:size(intervals,1)
                if(intervals(count,2) - intervals(count,1) < obj.minRayLengths(free_variable_index))
                    intervals(count,:) = [];
                else
                    count = count+1;
                end
            end
        end
    end
    
    methods (Access = private)
        % Method for evaluating in the case of degree of redundancy = 1.
        function intervals = evaluate_function_fully_restrained(obj,model,workspace_ray,maximum_degree,free_variable_index,free_variable_linear_space,least_squares_matrix_i)
            % Determine all of the sets of combinatorics that will be used
            q_zero = zeros(obj.numDofs, 1);
            cable_vector = 1:obj.numCables;
            cable_combinations = zeros(obj.numCables,obj.numDofs);
            for i = 1:obj.numCables
                temp_cable = cable_vector; temp_cable(i) = [];
                cable_combinations(i,:) = temp_cable;
            end
            
            % Pose data
            q_fixed = workspace_ray.fixedVariables;
            fixed_index = true(obj.numDofs,1); fixed_index(workspace_ray.freeVariableIndex) = false;
            q = q_zero; 
            q(fixed_index) = q_fixed;
            %% Sample the polynomials
            % Start with matrix initialisation
            null_matrix = zeros(maximum_degree+1,obj.numCables);
            for linear_space_index = 1:maximum_degree+1
                % Update the value for q
                q_free = free_variable_linear_space(linear_space_index);
                q(free_variable_index) = q_free;
                % Update the model
                model.update(q, q_zero, q_zero, q_zero);
                % Obtain the Jacobian for computation
                if(obj.areDofsTranslation(free_variable_index))
                    A = -(model.L)';
                else
                    A = - (1+tan(0.5*q_free)^2)*(model.L)'; % Scalar multiplication is to multiply out the denominator of the Weierstrauss substitution
                end
                % Scale the Jacobian by the cable lengths (to remove the
                % denominator)
                A = A*diag(model.cableLengths);
                % Set up all of the components
                for combination_index = 1:obj.numCables
                    A_comb = A(:,cable_combinations(combination_index,:));
                    null_matrix(linear_space_index,combination_index) = det(A_comb);
                end
            end
            %% Determination of intervals
            intervals = [];
            polynomial_coefficients_null = zeros(obj.numCables,maximum_degree+1); % Initialised once it is always completely updated
            sign_vector = zeros(obj.numCables,1);
            null_roots = [workspace_ray.freeVariableRange(1);workspace_ray.freeVariableRange(2)];
            for combination_index = 1:obj.numCables
                % Repeat for each combination and k
                null_vector = null_matrix(:,combination_index);
                polynomial_coefficients_null(combination_index,:) = ((-1)^(combination_index+1))*(least_squares_matrix_i*null_vector);
                coefficients_null = polynomial_coefficients_null(combination_index,:);
                leading_zero_number = -1;
                % Remove the leading zeros
                for i = 1:maximum_degree+1
                    if(abs(coefficients_null(i)) > obj.TOLERANCE)
                        leading_zero_number = i-1;
                        break;
                    end
                end
                if(leading_zero_number ~= -1)
                    coefficients_null(1:leading_zero_number) = [];
                    null_i_roots = roots(coefficients_null);
                    % Remove roots that are complex
                    null_i_roots = null_i_roots(imag(null_i_roots)==0);
                    % If rotation convert back to angle
                    if(~obj.areDofsTranslation(free_variable_index))
                        null_i_roots = 2*atan(null_i_roots);
                    end
                    % Remove roots that lie outside of the range
                    null_i_roots(null_i_roots<workspace_ray.freeVariableRange(1)) = [];
                    null_i_roots(null_i_roots>workspace_ray.freeVariableRange(2)) = [];
                    % incorporate the roots into the roots for all k
                    null_roots = [null_roots;null_i_roots];
                end
            end
            % sort the roots
            null_roots = sort(null_roots);
            % go through all of the roots and check if at the
            % midpoints they have the same sign
            for root_index=1:length(null_roots)-1
                evaluation_interval = [null_roots(root_index),null_roots(root_index+1)];
                % Take the mean value of the interval
                if(obj.areDofsTranslation(free_variable_index))
                    mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                else
                    mean_value=tan(0.25*(evaluation_interval(2) + evaluation_interval(1)));
                end
                % Check the sign
                poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                for cable_index=1:obj.numCables
                    sign_vector(cable_index)=polynomial_coefficients_null(cable_index,:)*poly_vector;
                end
                if((sum(sign_vector>obj.TOLERANCE) == obj.numCables)||(sum(sign_vector<-obj.TOLERANCE) == obj.numCables))
                    % Add the interval
                    new_interval = evaluation_interval;
                    intervals = obj.set_union(intervals, new_interval);
                end
            end
            
            % Stop if the interval is the whole set
            if((~isempty(intervals))&&((abs(intervals(1,1) - workspace_ray.freeVariableRange(1)) < obj.TOLERANCE) && (abs(intervals(1,2) - workspace_ray.freeVariableRange(2))<obj.TOLERANCE)))
                return;
            end
        end
        
        % Method for evaluating inthe case of degree of redundnacy > 1
        function intervals = evaluate_function_redundantly_restrained(obj,model,workspace_ray,maximum_degree,free_variable_index,free_variable_linear_space,least_squares_matrix_i)
            q_zero = zeros(obj.numDofs, 1);
            % Determine all of the sets of combinatorics that will be used
            cable_vector = 1:obj.numCables;
            cable_combinations = nchoosek(cable_vector,obj.numDofs);
            number_combinations = size(cable_combinations,1);
            
            number_secondary_combinations = 2^obj.degRedundancy - 1;
            % Pose data
            q_fixed = workspace_ray.fixedVariables;
            fixed_index = true(obj.numDofs,1); fixed_index(workspace_ray.freeVariableIndex) = false;
            q = q_zero; 
            q(fixed_index) = q_fixed;
            %% Sample the polynomials
            % Start with matrix initialisation
            determinant_matrix = zeros(maximum_degree+1,number_combinations);
            null_matrix = zeros(maximum_degree+1,number_combinations,number_secondary_combinations,obj.numDofs+1);
            for linear_space_index = 1:maximum_degree+1
                % Update the value for q
                q_free = free_variable_linear_space(linear_space_index);
                q(free_variable_index) = q_free;
                % Update the model
                model.update(q, q_zero, q_zero, q_zero);
                % Obtain the Jacobian for computation
                if(obj.areDofsTranslation(free_variable_index))
                    A = -(model.L)';
                else
                    A = -(1+tan(0.5*q_free)^2)*(model.L)'; % Scalar multiplication is to multiply out the denominator of the Weierstrauss substitution
                end
                % Scale the Jacobian by the cable lengths (to remove the
                % denominator)
                A = A*diag(model.cableLengths);
                % Set up all of the components
                for combination_index = 1:number_combinations
                    A_comb = A(:,cable_combinations(combination_index,:));
                    determinant_matrix(linear_space_index,combination_index) = det(A_comb);
                    secondary_combination_index = 0;
                    temp_cable_vector = cable_vector; temp_cable_vector(cable_combinations(combination_index,:)) = [];                        
                    for combination_index_2 = 1:obj.degRedundancy
                        % Extract the combinations
                        if(combination_index_2 == obj.degRedundancy)
                            secondary_combinations_matrix = temp_cable_vector;
                        else
                            secondary_combinations_matrix = nchoosek(temp_cable_vector,combination_index_2);
                        end
                        for secondary_combinations_index_2 = 1:size(secondary_combinations_matrix,1)
                            secondary_combination_index = secondary_combination_index+1;
                            % Create the combined matrix
                            if(combination_index_2 == 1)
                                A_np1 = [A_comb,A(:,secondary_combinations_matrix)];
                            else
                                A_np1 = [A_comb,sum(A(:,secondary_combinations_matrix(secondary_combinations_index_2,:)),2)];
                            end
                            % Go through it and fill in all the
                            % determinants
                            for dof_iterations=1:obj.numDofs+1
                                temp_true = true(obj.numDofs+1,1);
                                temp_true(dof_iterations) = false;
                                null_matrix(linear_space_index,combination_index,secondary_combination_index,dof_iterations) = det(A_np1(:,temp_true));
                            end
                        end
                    end
                end
            end
            %% Determinant root combinations
            % Determine the polynomials and roots 
            polynomial_coefficients_det = zeros(maximum_degree+1,number_combinations);
            roots_cell_array = cell(number_combinations,1);
            leading_zero_number = -1*ones(number_combinations,1);
            intervals = [];
            for combination_index = 1:number_combinations
                determinant_vector = determinant_matrix(:,combination_index);
                polynomial_coefficients_det(:,combination_index) = least_squares_matrix_i*determinant_vector;
                
                % Find the roots
                % First remove anything that has magnitude below tolerance
                coefficients_det = polynomial_coefficients_det(:,combination_index);
                for i = 1:maximum_degree+1
                    if(abs(coefficients_det(i))>obj.TOLERANCE)
                        leading_zero_number(combination_index) = i-1;
                        break;
                    end
                end
                if(leading_zero_number(combination_index) ~= -1)
                    coefficients_det(1:leading_zero_number(combination_index)) = [];
                    temp_roots = roots(coefficients_det);
                    % Remove roots that are complex
                    temp_roots = temp_roots(imag(temp_roots)==0);
                    % If rotation convert back to angle
                    if(~obj.areDofsTranslation(free_variable_index))
                        temp_roots = 2*atan(temp_roots);
                    end
                    % Remove roots that lie outside of the range
                    temp_roots(temp_roots<workspace_ray.freeVariableRange(1)) = [];
                    temp_roots(temp_roots>workspace_ray.freeVariableRange(2)) = [];
                    roots_cell_array{combination_index} = sort(temp_roots);
                end
            end
            %% Determination of intervals
            polynomial_coefficients_null = zeros(obj.numDofs+1,maximum_degree+1); % Initialised once it is always completely updated
            sign_vector = zeros(obj.numDofs+1,1);
            for combination_index = 1:number_combinations
                if(leading_zero_number(combination_index) ~= -1)
                    % Repeat for each combination and k
                    for combination_index_2 = 1:number_secondary_combinations
                        null_roots = [workspace_ray.freeVariableRange(1);workspace_ray.freeVariableRange(2)];
                        for dof_index = 1:obj.numDofs+1
                            null_vector = null_matrix(:,combination_index,combination_index_2,dof_index);
                            polynomial_coefficients_null(dof_index,:) = ((-1)^(dof_index+1))*(least_squares_matrix_i*null_vector);
                            coefficients_null = polynomial_coefficients_null(dof_index,:);
                            if((sum(isinf(coefficients_null))==0)&&(sum(isnan(coefficients_null))==0))
                                null_i_roots = roots(coefficients_null);
                                % Remove roots that are complex
                                null_i_roots = null_i_roots(imag(null_i_roots)==0);
                                % If rotation convert back to angle
                                if(~obj.areDofsTranslation(free_variable_index))
                                    null_i_roots = 2*atan(null_i_roots);
                                end
                                % Remove roots that lie outside of the range
                                null_i_roots(null_i_roots<workspace_ray.freeVariableRange(1)) = [];
                                null_i_roots(null_i_roots>workspace_ray.freeVariableRange(2)) = [];
                                % incorporate the roots into the roots for all k
                                null_roots = [null_roots;null_i_roots];
                            end
                        end
                        % sort the roots
                        null_roots = sort(null_roots);
%                         if any(ismember(round(null_roots,2), -0.68))
%                             null_roots
%                         end
                        % go through all of the roots and check if at the
                        % midpoints they have the same sign
                        for root_index=1:length(null_roots)-1
                            evaluation_interval = [null_roots(root_index),null_roots(root_index+1)];
                            roots_list_ij = sort([roots_cell_array{combination_index};roots_cell_array{combination_index_2}]);
                            number_determinant_roots = length(roots_list_ij);
                            if(number_determinant_roots > 0)
                                for root_index_2 = 1:number_determinant_roots
                                    root_ij = roots_list_ij(root_index_2);
                                    if((root_ij > evaluation_interval(1))&&(root_ij < evaluation_interval(2)))
                                        % The root is within the evaluation
                                        % interval
                                        evaluation_interval(2) = root_ij - obj.TOLERANCE;
                                        % Take the mean value of the interval
                                        if(obj.areDofsTranslation(free_variable_index))
                                            mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                        else
                                            mean_value=tan(0.25*(evaluation_interval(2) + evaluation_interval(1)));
                                        end
                                        % Check the sign
                                        poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree); 
                                        for dof_index=1:obj.numDofs+1
                                            sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                        end
                                        if((sum(sign_vector>obj.TOLERANCE) == obj.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.numDofs+1))
                                            % Add the interval
                                            new_interval = evaluation_interval;
                                            intervals = obj.set_union(intervals,new_interval);
                                        end
                                        % Update the evaluation interval
                                        evaluation_interval(1) = evaluation_interval(2) + 2*obj.TOLERANCE;
                                        evaluation_interval(2) = null_roots(root_index+1);
                                        if(root_index_2 == number_determinant_roots)
                                            % Evaluate the final
                                            % interval if this is the
                                            % last root
                                            % Take the mean value of the interval
                                            if(obj.areDofsTranslation(free_variable_index))
                                                mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                            else
                                                mean_value=tan(0.25*(evaluation_interval(2) + evaluation_interval(1)));
                                            end
                                            % Check the sign
                                            poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                                            for dof_index=1:obj.numDofs+1
                                                sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                            end
                                            if((sum(sign_vector>obj.TOLERANCE) == obj.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.numDofs+1))
                                                % Add the interval
                                                new_interval = evaluation_interval;
                                                intervals = obj.set_union(intervals,new_interval);
                                            end
                                        end
                                    elseif(root_ij >= evaluation_interval(2))
                                        % Evaluate
                                        if(obj.areDofsTranslation(free_variable_index))
                                            mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                        else
                                            mean_value=tan(0.25*(evaluation_interval(2) + evaluation_interval(1)));
                                        end
                                        % Check the sign
                                        poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                                        for dof_index=1:obj.numDofs+1
                                            sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                        end
                                        if((sum(sign_vector>obj.TOLERANCE) == obj.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.numDofs+1))
                                            % Add the interval
                                            new_interval = evaluation_interval;
                                            intervals = obj.set_union(intervals,new_interval);
                                        end
                                        break;
                                    end
                                end
                            else
                                % Take the mean value of the interval
                                if(obj.areDofsTranslation(free_variable_index))
                                    mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                else
                                    mean_value=tan(0.25*(evaluation_interval(2) + evaluation_interval(1)));
                                end
                                % Check the sign
                                poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree); 
                                for dof_index=1:obj.numDofs+1
                                    sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                end
                                if((sum(sign_vector>obj.TOLERANCE) == obj.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.numDofs+1))
                                    new_interval = evaluation_interval;
                                    intervals = obj.set_union(intervals,new_interval);
                                end
                            end
                        end
                    end
                    % Stop if the interval is the whole set
                    if((~isempty(intervals))&&((abs(intervals(1,1) - workspace_ray.freeVariableRange(1)) < obj.TOLERANCE) && (abs(intervals(1,2) - workspace_ray.freeVariableRange(2))<obj.TOLERANCE)))
                        return;
                    end
                end
                if (~isempty(intervals) && (obj.degRedundancy==1) && (intervals(1,2) - intervals(1,1) > obj.minRayLengths(free_variable_index)))
                    return;
                end
            end
        end
        
        % The method for taking the union of different intervals
        function union_set = set_union(obj,interval_set,interval)
            number_intervals = size(interval_set,1);
            interval_min = interval(1); interval_max = interval(2);
            new_interval = true;
            for interval_index = 1:number_intervals
                interval_set_i_min = interval_set(interval_index,1);
                interval_set_i_max = interval_set(interval_index,2);
                % Determine the interval that it is overlapping with 
                if((interval_min - interval_set_i_min <= obj.TOLERANCE)&&(interval_set_i_max - interval_max <= obj.TOLERANCE))
                    % The interval contains an existing interval
                    interval_set(interval_index,1) = interval_min;
                    interval_set(interval_index,2) = interval_max;
                    % Check if this can be combined with an existing
                    % element
                    interval_set = obj.set_union([interval_set(1:interval_index-1,:);interval_set(interval_index+1:number_intervals,:)],interval_set(interval_index,:));
                    new_interval = false;
                    break;
                elseif((interval_min - interval_set_i_max <= obj.TOLERANCE)&&(interval_max - interval_set_i_max > obj.TOLERANCE))
                    % Interval should be extended on the max side
                    interval_set(interval_index,2) = interval_max;
                    % Check if this can be combined with an existing
                    % element
                    interval_set = obj.set_union([interval_set(1:interval_index-1,:);interval_set(interval_index+1:number_intervals,:)],interval_set(interval_index,:));
                    new_interval = false;
                    break;
                elseif((interval_set_i_min - interval_max <= obj.TOLERANCE)&&(interval_set_i_min - interval_min > obj.TOLERANCE))
                    % Interval should be extended on the min side
                    interval_set(interval_index,1) = interval_min;
                    % Check if this can be combined with an existing
                    % element
                    interval_set = obj.set_union([interval_set(1:interval_index-1,:);interval_set(interval_index+1:number_intervals,:)],interval_set(interval_index,:));
                    new_interval = false;
                    break;
                elseif((interval_max - interval_set_i_max <= obj.TOLERANCE)&&(interval_set_i_min - interval_min <= obj.TOLERANCE))
                    % Interval is contained within another interval
                    new_interval = false;
                    break;
                end
            end
            if(new_interval)
                interval_set(number_intervals+1,:) = interval;
            end
            union_set = interval_set;
        end
    end
end