% Class to compute
% Author        : Paul Cheng
% Created       : 2020
% Description   :

classdef WrenchClosureReconfigCondition < ReconfigConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 9;
        TOLERANCE = 1e-8;
        % Type of workspace condition (WorkspaceConditionType enum)
        type = ReconfigConditionType.WRENCH_CLOSURE;
        
    end
    properties (SetAccess = protected)
        model
        degRedundancy
    end
    
    methods
        % Constructor for interference free worksapce
        function w = WrenchClosureReconfigCondition(model)
            w.model = model;
            w.degRedundancy = model.numCables - model.numDofs;
        end
        
        function feasibleInterval =  evaluateFunction(obj, reconfig_element)
            tmp_model = obj.model;
            q_zeros = zeros(size(obj.model.q));
            tmp_model.update(reconfig_element.q,q_zeros,q_zeros,q_zeros);
            %            [obj.model,~] = obj.updateModelOA(obj.model,reconfig_element.OA_end,reconfig_element.q)
            if(obj.degRedundancy == 1)
                feasibleInterval = obj.evaluate_function_fully_restrained(reconfig_element);
            else
                feasibleInterval = obj.evaluate_function_redundantly_restrained(reconfig_element);
            end
            
        end
    end
    
    methods (Access = private)
        function intervals = evaluate_function_fully_restrained(obj,reconfig_element)
            % Determine all of the sets of combinatorics that will be used
            OA_0 = obj.model.cableModel.r_OAs(1:3,:);
            
            q_zero = zeros(obj.model.numDofs, 1);
            cable_vector = 1:obj.model.numCables;
            cable_combinations = zeros(obj.model.numCables,obj.model.numDofs);
            for i = 1:obj.model.numCables
                temp_cable = cable_vector; temp_cable(i) = [];
                cable_combinations(i,:) = temp_cable;
            end
            maximum_degree = 2;
            linear_space_var = linspace(0,1,maximum_degree+1);
            least_squares_matrix = GeneralMathOperations.ComputeLeastSquareMatrix(linear_space_var,maximum_degree);
            least_squares_matrix_i = inv(least_squares_matrix);
            
            q = reconfig_element.q;
            intervals = cell(obj.model.numCables,1);
            single_cable_interval = [];
            
            for cable_id = 1:obj.model.numCables
                if norm(reconfig_element.OA_begin(:,cable_id) - reconfig_element.OA_end(:,cable_id)) ~= 0
                    %% Sample the polynomials
                    % Start with matrix initialisation
                    null_matrix = zeros(maximum_degree+1,obj.model.numCables);
                    for linear_space_index = 1:maximum_degree+1
                        OA = OA_0;
                        OA(:,cable_id) = reconfig_element.OA_begin(:,cable_id) + linear_space_var(linear_space_index)*(reconfig_element.OA_end(:,cable_id) -  reconfig_element.OA_begin(:,cable_id));
                        
                        % Update the model and Obtain the Jacobian for computation
                        [obj.model,A] = obj.updateModelOA(obj.model,OA,q);
                        
                        
                        
                        % Scale the Jacobian by the cable lengths (to remove the
                        % denominator)
                        A = A*diag(obj.model.cableLengths);
                        % Set up all of the components
                        for combination_index = 1:obj.model.numCables
                            A_comb = A(:,cable_combinations(combination_index,:));
                            null_matrix(linear_space_index,combination_index) = det(A_comb);
                        end
                    end
                    [obj.model,~] = obj.updateModelOA(obj.model,OA_0,q);
                    %% Determination of intervals
                    
                    polynomial_coefficients_null = zeros(obj.model.numCables,maximum_degree+1); % Initialised once it is always completely updated
                    sign_vector = zeros(obj.model.numCables,1);
                    null_roots = [0;1];
                    %
                    for combination_index = 1:obj.model.numCables
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
                            
                            % Remove roots that lie outside of the range
                            null_i_roots(null_i_roots<0) = [];
                            null_i_roots(null_i_roots>1) = [];
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
                        mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                        
                        % Check the sign
                        poly_vector = GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                        for cable_index=1:obj.model.numCables
                            sign_vector(cable_index) = polynomial_coefficients_null(cable_index,:)*poly_vector;
                        end
                        if((sum(sign_vector>obj.TOLERANCE) == obj.model.numCables)||(sum(sign_vector<-obj.TOLERANCE) == obj.model.numCables))
                            % Add the interval
                            new_interval = evaluation_interval;
                            single_cable_interval = obj.set_union(single_cable_interval, new_interval);
                        end
                    end
                    if ~isempty(single_cable_interval)
                        for interval_id = 1:size(single_cable_interval,1)
                            intervals{cable_id}{interval_id} =  reconfig_element.OA_begin(:,cable_id) + single_cable_interval(interval_id,:).*(reconfig_element.OA_end(:,cable_id) -  reconfig_element.OA_begin(:,cable_id));
                        end
                    end
                else
                     intervals{cable_id}{1} = [obj.model.cableModel.r_OAs(1:3,cable_id),obj.model.cableModel.r_OAs(1:3,cable_id)];
                end
            end
        end
        
        function intervals = evaluate_function_redundantly_restrained(obj,reconfig_element)
            % Determine all of the sets of combinatorics that will be used
            OA_0 = obj.model.cableModel.r_OAs(1:3,:);
            
            q_zero = zeros(obj.model.numDofs, 1);
            cable_vector = 1:obj.model.numCables;
            cable_combinations = nchoosek(cable_vector,obj.model.numDofs);
            number_combinations = size(cable_combinations,1);
            number_secondary_combinations = 2^obj.degRedundancy - 1;
            
            maximum_degree = 2;
            linear_space_var = linspace(0,1,maximum_degree+1);
            least_squares_matrix = GeneralMathOperations.ComputeLeastSquareMatrix(linear_space_var,maximum_degree);
            least_squares_matrix_i = inv(least_squares_matrix);
            
            q = reconfig_element.q;
            intervals = cell(obj.model.numCables,1);
            single_cable_interval = [];
            
            for cable_id = 1:obj.model.numCables
                if norm(reconfig_element.OA_begin(:,cable_id) - reconfig_element.OA_end(:,cable_id)) ~= 0
                    %% Sample the polynomials
                    % Start with matrix initialisation
                    determinant_matrix = zeros(maximum_degree+1,number_combinations);
                    null_matrix = zeros(maximum_degree+1,obj.model.numCables);
                    for linear_space_index = 1:maximum_degree+1
                        OA = OA_0;
                        OA(:,cable_id) = reconfig_element.OA_begin(:,cable_id) + linear_space_var(linear_space_index)*(reconfig_element.OA_end(:,cable_id) -  reconfig_element.OA_begin(:,cable_id));
                        
                        % Update the model and Obtain the Jacobian for computation
                        [obj.model,A] = obj.updateModelOA(obj.model,OA,q);
                        
                        % Scale the Jacobian by the cable lengths (to remove the
                        % denominator)
                        A = A*diag(obj.model.cableLengths);
                        % Set up all of the components
                        for combination_index = 1:number_combinations
                            A_comb = A(:,cable_combinations(combination_index,:));
                            null_matrix(linear_space_index,combination_index) = det(A_comb);
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
                                    for dof_iterations=1:obj.model.numDofs+1
                                        temp_true = true(obj.model.numDofs+1,1);
                                        temp_true(dof_iterations) = false;
                                        null_matrix(linear_space_index,combination_index,secondary_combination_index,dof_iterations) = det(A_np1(:,temp_true));
                                    end
                                end
                            end
                        end
                    end
                    [obj.model,~] = obj.updateModelOA(obj.model,OA_0,q);
                    
                    %% Determinant root combinations
                    % Determine the polynomials and roots
                    polynomial_coefficients_det = zeros(maximum_degree+1,number_combinations);
                    roots_cell_array = cell(number_combinations,1);
                    leading_zero_number = -1*ones(number_combinations,1);
                    
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
                            
                            % Remove roots that lie outside of the range
                            temp_roots(temp_roots<0) = [];
                            temp_roots(temp_roots>1) = [];
                            roots_cell_array{combination_index} = sort(temp_roots);
                        end
                    end
                    %% Determination of intervals
                    
                    polynomial_coefficients_null = zeros(obj.model.numCables,maximum_degree+1); % Initialised once it is always completely updated
                    sign_vector = zeros(obj.model.numCables,1);
                    %                 null_roots = [0;1];
                    for combination_index = 1:number_combinations
                        if(leading_zero_number(combination_index) ~= -1)
                            % Repeat for each combination and k
                            for combination_index_2 = 1:number_secondary_combinations
                                null_roots = [0;1];
                                for dof_index = 1:obj.model.numDofs+1
                                    null_vector = null_matrix(:,combination_index,combination_index_2,dof_index);
                                    polynomial_coefficients_null(dof_index,:) = ((-1)^(dof_index+1))*(least_squares_matrix_i*null_vector);
                                    coefficients_null = polynomial_coefficients_null(dof_index,:);
                                    if((sum(isinf(coefficients_null))==0)&&(sum(isnan(coefficients_null))==0))
                                        null_i_roots = roots(coefficients_null);
                                        % Remove roots that are complex
                                        null_i_roots = null_i_roots(imag(null_i_roots)==0);
                                        
                                        % Remove roots that lie outside of the range
                                        null_i_roots(null_i_roots<0) = [];
                                        null_i_roots(null_i_roots>1) = [];
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
                                                mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                                % Check the sign
                                                poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                                                for dof_index=1:obj.model.numDofs+1
                                                    sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                                end
                                                if((sum(sign_vector>obj.TOLERANCE) == obj.model.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.model.numDofs+1))
                                                    % Add the interval
                                                    new_interval = evaluation_interval;
                                                    single_cable_interval = obj.set_union(single_cable_interval,new_interval);
                                                end
                                                % Update the evaluation interval
                                                evaluation_interval(1) = evaluation_interval(2) + 2*obj.TOLERANCE;
                                                evaluation_interval(2) = null_roots(root_index+1);
                                                if(root_index_2 == number_determinant_roots)
                                                    % Evaluate the final
                                                    % interval if this is the
                                                    % last root
                                                    % Take the mean value of the interval
                                                    mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                                    % Check the sign
                                                    poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                                                    for dof_index=1:obj.model.numDofs+1
                                                        sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                                    end
                                                    if((sum(sign_vector>obj.TOLERANCE) == obj.model.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.model.numDofs+1))
                                                        % Add the interval
                                                        new_interval = evaluation_interval;
                                                        single_cable_interval = obj.set_union(single_cable_interval,new_interval);
                                                    end
                                                end
                                            elseif(root_ij >= evaluation_interval(2))
                                                % Evaluate
                                                mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                                % Check the sign
                                                poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                                                for dof_index=1:obj.model.numDofs+1
                                                    sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                                end
                                                if((sum(sign_vector>obj.TOLERANCE) == obj.model.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.model.numDofs+1))
                                                    % Add the interval
                                                    new_interval = evaluation_interval;
                                                    single_cable_interval = obj.set_union(single_cable_interval,new_interval);
                                                end
                                                break;
                                            end
                                        end
                                    else
                                        % Take the mean value of the interval
                                        mean_value = 0.5*(evaluation_interval(2) + evaluation_interval(1));
                                        % Check the sign
                                        poly_vector=GeneralMathOperations.ComputePolynomialVector(mean_value,maximum_degree);
                                        for dof_index=1:obj.model.numDofs+1
                                            sign_vector(dof_index)=polynomial_coefficients_null(dof_index,:)*poly_vector;
                                        end
                                        if((sum(sign_vector>obj.TOLERANCE) == obj.model.numDofs+1)||(sum(sign_vector<-obj.TOLERANCE) == obj.model.numDofs+1))
                                            new_interval = evaluation_interval;
                                            single_cable_interval = obj.set_union(single_cable_interval,new_interval);
                                        end
                                    end
                                end
                            end
                            %                     % Stop if the interval is the whole set
                            %                     if((~isempty(single_cable_interval))&&((abs(single_cable_interval(1,1) - 0) < obj.TOLERANCE) && (abs(single_cable_interval(1,2) - 1)<obj.TOLERANCE)))
                            %                         return;
                            %                     end
                        end
                        %                 if (~isempty(single_cable_interval) && (obj.degRedundancy==1) && (single_cable_interval(1,2) - single_cable_interval(1,1) > obj.minRayLengths(free_variable_index)))
                        %                     return;
                        %                 end
                    end
                    if ~isempty(single_cable_interval)
                        for interval_id = 1:size(single_cable_interval,1)
                            intervals{cable_id}{interval_id} =  reconfig_element.OA_begin(:,cable_id) + single_cable_interval(interval_id,:).*(reconfig_element.OA_end(:,cable_id) -  reconfig_element.OA_begin(:,cable_id));
                        end
                    end
                else
                    intervals{cable_id}{1} = [obj.model.cableModel.r_OAs(1:3,cable_id),obj.model.cableModel.r_OAs(1:3,cable_id)];
                end
            end
        end
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
    methods (Static)
        function [model,L] = updateModelOA(model,OA_new,q)
            for j = 1:model.numCables
                model.cableModel.cables{j}.attachments{1}.updateAttachmentLocation(OA_new(:,j),'COM');
            end
            q_zeros = zeros(size(model.q));
            model.update(q,q_zeros,q_zeros,q_zeros);
            L = -model.L';
            
        end
        
        
    end
    
end