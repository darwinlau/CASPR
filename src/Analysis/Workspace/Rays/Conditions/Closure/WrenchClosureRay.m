% Class to compute whether a pose (dynamics) is within the wrench-closure
% workspace (WCW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
classdef WrenchClosureRay < WorkspaceRayConditionBase
    properties (SetAccess = protected, GetAccess = protected)
        min_ray_percentage          % The minimum percentage of the ray at which it is included
        tolerance = 1e-8;
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchClosureRay(min_ray_percent)
            w.min_ray_percentage = min_ray_percent;
        end
        
        % The new evaluate function
        function intervals = evaluateFunction(obj,model,workspace_ray)
            % Variable Initialisation
            number_dofs = model.numDofs;
            number_cables = model.numCables;
            joint_type = model.bodyModel.q_dofType;
            free_variable_index = workspace_ray.free_variable_index;
            zero_n = zeros(number_dofs,1);
            % Use the joint type to determine the maximum polynomial
            % degrees
            if(joint_type(free_variable_index)==DoFType.TRANSLATION)
                maximum_degree = number_dofs;
            else
                maximum_degree = 2*number_dofs;
            end
            % Set up a linear space for the free variable
            free_variable_linear_space = linspace(workspace_ray.free_variable_range(1),workspace_ray.free_variable_range(2),maximum_degree+1);
            % Use the joint type to determine the maximum polynomial
            % degrees
            if(joint_type(free_variable_index)==DoFType.TRANSLATION)
                least_squares_matrix = GeneralMathOperations.ComputeLeastSquareMatrix(free_variable_linear_space',maximum_degree);
            else
                least_squares_matrix = GeneralMathOperations.ComputeLeastSquareMatrix(tan(0.5*free_variable_linear_space)',maximum_degree);
            end
            % Determine all of the sets of combinatorics that will be used
            cable_combinations = nchoosek(1:number_cables,number_dofs);
            number_combinations = size(cable_combinations,1);
            % Determine the determinant vector (to be used for rank
            % computation)
            q_fixed = workspace_ray.fixed_variables;
            fixed_index = true(number_dofs,1); fixed_index(workspace_ray.free_variable_index) = false;
            q = zero_n; q(fixed_index) = q_fixed;
            determinant_matrix = zeros(maximum_degree+1,number_combinations);
            matrix_a_t = zeros(number_dofs,maximum_degree+1,number_combinations);
            matrix_k = zeros(number_dofs,maximum_degree+1,number_combinations,number_combinations);
            for linear_space_index = 1:maximum_degree+1
                q_free = free_variable_linear_space(linear_space_index);
                q(free_variable_index) = q_free;
                model.update(q,zero_n,zero_n,zero_n);
                % Obtain the Jacobian for computation
                if(joint_type(free_variable_index)==DoFType.TRANSLATION)
                    A = -(model.L)';
                else
                    A = - (1+tan(0.5*q_free)^2)*(model.L)'; % Scalar multiplication is to multiply out the denominator of the Weierstrauss substitution
                end
                % Scale the Jacobian by the cable lengths (to remove the
                % denominator)
                for cable_index = 1:number_cables
                    A(:,cable_index) = A(:,cable_index)*model.cableLengths(cable_index);
                end
                % Set up all of the components
                for combination_index = 1:number_combinations
                    A_comb = A(:,cable_combinations(combination_index,:));
                    determinant_matrix(linear_space_index,combination_index) = det(A_comb);
                    matrix_a_t(:,linear_space_index,combination_index) = -sum(A_comb,2); % a_t = -\Sum a_i
                    for combination_index_2 = 1:number_combinations
                        A_comb_2 = A(:,cable_combinations(combination_index_2,:));
                        matrix_k(:,linear_space_index,combination_index,combination_index_2) = det(A_comb_2)*(A_comb_2\matrix_a_t(:,linear_space_index,combination_index));
                    end
                end
            end
            % Determine the polynomials and roots 
            polynomial_coefficients_det = zeros(maximum_degree+1,number_combinations);
%             polynomial_coefficients_k = zeros(number_dofs,maximum_degree+1,number_combinations,number_combinations);
            roots_cell_array = cell(number_combinations,1);
            leading_zero_number = -1*ones(number_combinations,1);
            intervals = [];
            % THE ORDERING NEEDS TO BE CHANGED IN THIS VICINITY
            for combination_index = 1:number_combinations
                determinant_vector = determinant_matrix(:,combination_index);
                polynomial_coefficients_det(:,combination_index) = least_squares_matrix\determinant_vector;
%                 if(joint_type(free_variable_index)==DoFType.TRANSLATION)
%                     % Double check regarding the use of -1
% %                     polynomial_coefficients_det(:,combination_index) = polyfit(free_variable_linear_space,determinant_vector',maximum_degree);
%                     polynomial_coefficients_det(:,combination_index) = GeneralMathOperations.PolynomialFit(free_variable_linear_space',determinant_vector,maximum_degree)';
%                 else
% %                     polynomial_coefficients_det(:,combination_index) = polyfit(tan(0.5*free_variable_linear_space),determinant_vector',maximum_degree);
%                     polynomial_coefficients_det(:,combination_index) = GeneralMathOperations.PolynomialFit(tan(0.5*free_variable_linear_space)',determinant_vector,maximum_degree)';
%                 end
                % Find the roots
                % First remove anything that has magnitude below tolerance
                coefficients_det = polynomial_coefficients_det(:,combination_index);
                for i = 1:maximum_degree+1
                    if(abs(coefficients_det(i))>obj.tolerance)
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
                    if(joint_type(free_variable_index)~=DoFType.TRANSLATION)
                        temp_roots = 2*atan(temp_roots);
                    end
                    % Remove roots that lie outside of the range
                    temp_roots(temp_roots<workspace_ray.free_variable_range(1)) = [];
                    temp_roots(temp_roots>workspace_ray.free_variable_range(2)) = [];
                    roots_cell_array{combination_index} = sort(temp_roots);
                end
            end
            polynomial_coefficients_k = zeros(number_dofs,maximum_degree+1); % Initialised once it is always completely updated
            for combination_index = 1:number_combinations
                if(leading_zero_number(combination_index) ~= -1)
                    % Repeat for each combination and k
                    for combination_index_2 = 1:number_combinations
                        if(combination_index_2 ~= combination_index)
                            k_roots = [workspace_ray.free_variable_range(1);workspace_ray.free_variable_range(2)];
                            for dof_index = 1:number_dofs
                                k = matrix_k(dof_index,:,combination_index,combination_index_2);
                                polynomial_coefficients_k(dof_index,:) = least_squares_matrix\(k');
%                                 if(joint_type(free_variable_index)==DoFType.TRANSLATION)
% %                                     polynomial_coefficients_k(dof_index,:) = polyfit(free_variable_linear_space,k,maximum_degree);
%                                     polynomial_coefficients_k(dof_index,:) = GeneralMathOperations.PolynomialFit(free_variable_linear_space',k',maximum_degree)';
%                                 else
% %                                     polynomial_coefficients_k(dof_index,:) = polyfit(tan(0.5*free_variable_linear_space),k,maximum_degree);
%                                     polynomial_coefficients_k(dof_index,:) = GeneralMathOperations.PolynomialFit(tan(0.5*free_variable_linear_space)',k',maximum_degree)';
%                                 end
                                coefficients_k_i = polynomial_coefficients_k(dof_index,:);
                                if((sum(isinf(coefficients_k_i))==0)&&(sum(isnan(coefficients_k_i))==0))
                                    k_i_roots = roots(coefficients_k_i);
                                    % Remove roots that are complex
                                    k_i_roots = k_i_roots(imag(k_i_roots)==0);
                                    % If rotation convert back to angle
                                    if(joint_type(free_variable_index)~=DoFType.TRANSLATION)
                                        k_i_roots = 2*atan(k_i_roots);
                                    end
                                    % Remove roots that lie outside of the range
                                    k_i_roots(k_i_roots<workspace_ray.free_variable_range(1)) = [];
                                    k_i_roots(k_i_roots>workspace_ray.free_variable_range(2)) = [];
                                    % incorporate the roots into the roots for all k
                                    k_roots = [k_roots;k_i_roots];
                                end
                            end
                            % sort the roots
                            k_roots = sort(k_roots);
                            % go through all of the roots and check if at the
                            % midpoints they have the same sign
                            for root_index=1:length(k_roots)-1
                                evaluation_interval = [k_roots(root_index),k_roots(root_index+1)];
                                roots_list_ij = sort([roots_cell_array{combination_index};roots_cell_array{combination_index_2}]);
                                number_determinant_roots = length(roots_list_ij);
                                if(number_determinant_roots > 0)
                                    for root_index_2 = 1:number_determinant_roots
                                        root_ij = roots_list_ij(root_index_2);
                                        if((root_ij > evaluation_interval(1))&&(root_ij < evaluation_interval(2)))
                                            % The root is within the evaluation
                                            % interval
                                            evaluation_interval(2) = root_ij - obj.tolerance;
                                            % Take the mean value of the interval
                                            if(joint_type(free_variable_index)==DoFType.TRANSLATION)
                                                mean_value = mean(evaluation_interval);
                                            else
                                                mean_value=tan(mean(evaluation_interval)/2);
                                            end
                                            % Check the sign
                                            sign_vector=zero_n;
                                            sign_det = sign(polyval(polynomial_coefficients_det(:,combination_index_2),mean_value));
                                            for dof_index=1:number_dofs
                                                sign_vector(dof_index)=polyval(polynomial_coefficients_k(dof_index,:),mean_value);
                                            end
                                            sign_vector = sign_det*sign_vector;
                                            if((sum(sign_vector>obj.tolerance) == number_dofs))
                                                % Add the interval
                                                new_interval = evaluation_interval;
                                                intervals = obj.set_union(intervals,new_interval);
                                            end
                                            % Update the evaluation interval
                                            evaluation_interval(1) = evaluation_interval(2) + 2*obj.tolerance;
                                            evaluation_interval(2) = k_roots(root_index+1);
                                            if(root_index_2 == number_determinant_roots)
                                                % Evaluate the final
                                                % interval if this is the
                                                % last root
                                                % Take the mean value of the interval
                                                if(joint_type(free_variable_index)==DoFType.TRANSLATION)
                                                    mean_value = mean(evaluation_interval);
                                                else
                                                    mean_value=tan(mean(evaluation_interval)/2);
                                                end
                                                % Check the sign
                                                sign_vector=zero_n;
                                                sign_det = sign(polyval(polynomial_coefficients_det(:,combination_index_2),mean_value));
                                                for dof_index=1:number_dofs
                                                    sign_vector(dof_index)=polyval(polynomial_coefficients_k(dof_index,:),mean_value);
                                                end
                                                sign_vector = sign_det*sign_vector;
                                                if((sum(sign_vector>obj.tolerance) == number_dofs))
                                                    % Add the interval
                                                    new_interval = evaluation_interval;
                                                    intervals = obj.set_union(intervals,new_interval);
                                                end
                                            end
                                        elseif(root_ij >= evaluation_interval(2))
                                            % Evaluate
                                            if(joint_type(free_variable_index)==DoFType.TRANSLATION)
                                                mean_value = mean(evaluation_interval);
                                            else
                                                mean_value=tan(mean(evaluation_interval)/2);
                                            end
                                            % Check the sign
                                            sign_vector=zero_n;
                                            sign_det = sign(polyval(polynomial_coefficients_det(:,combination_index_2),mean_value));
                                            for dof_index=1:number_dofs
                                                sign_vector(dof_index)=polyval(polynomial_coefficients_k(dof_index,:),mean_value);
                                            end
                                            sign_vector = sign_det*sign_vector;
                                            if((sum(sign_vector>obj.tolerance) == number_dofs))
                                                % Add the interval
                                                new_interval = evaluation_interval;
                                                intervals = obj.set_union(intervals,new_interval);
                                            end
                                            break;
                                        end
                                    end
                                else
                                    % Take the mean value of the interval
                                    if(joint_type(free_variable_index)==DoFType.TRANSLATION)
                                        mean_value = mean(evaluation_interval);
                                    else
                                        mean_value=tan(mean(evaluation_interval)/2);
                                    end
                                    % Check the sign
                                    sign_vector=zero_n;
                                    sign_det = sign(polyval(polynomial_coefficients_det(:,combination_index_2),mean_value));
                                    for dof_index=1:number_dofs
                                        sign_vector(dof_index)=polyval(polynomial_coefficients_k(dof_index,:),mean_value);
                                    end
                                    sign_vector = sign_det*sign_vector;
                                    if((sum(sign_vector>obj.tolerance) == number_dofs))
                                        new_interval = evaluation_interval;
                                        intervals = obj.set_union(intervals,new_interval);
                                    end
                                end
                            end
                        end
                    end
                    % Stop if the interval is the whole set
                    if((~isempty(intervals))&&((abs(intervals(1,1) - workspace_ray.free_variable_range(1)) < obj.tolerance) && (abs(intervals(1,2) - workspace_ray.free_variable_range(2))<obj.tolerance)))
                        return;
                    end
                end
            end
            count = 1;
            for iteration_index = 1:size(intervals,1)
                segment_percentage = 100*(intervals(count,2)-intervals(count,1))/(workspace_ray.free_variable_range(2) - workspace_ray.free_variable_range(1));
                if(segment_percentage < obj.min_ray_percentage)
                    intervals(count,:) = [];
                else
                    count = count+1;
                end
            end
        end
        
%         % Evaluate the wrench closure condition return true if satisfied 
%         function intervals = evaluateFunction(obj,model,workspace_ray)
%             zeroval=1e-8; % THIS COULD BE MADE AN OPTION FOR THE SOLVER
%             % Variable initialisation
%             numDofs=model.numDofs;
%             numCables=model.numCables;
%             curtypevar= model.bodyModel.q_dofType;
%             curflexvar = workspace_ray.free_variable_index;
%             % Use the joint type to determine the maximum polynomial degree
%             if curtypevar(curflexvar)==DoFType.TRANSLATION
%                 maxdeg=numDofs;
%             else
%                 maxdeg=2*numDofs;
%             end
%             % Set up a lin space for the free variable
%             flxvarlinspace=linspace(workspace_ray.free_variable_range(1),workspace_ray.free_variable_range(2),maxdeg+1);
%             % Determine combinatoric variables
%             cab_comb=nchoosek(1:numCables,numDofs+1);
%             [numcomb,~]=size(cab_comb);
%             % Compute matf
%             matf=zeros(maxdeg+1,numcomb*(numDofs+1));
%             % Set up the pose vector
%             q_fixed = workspace_ray.fixed_variables;
%             fixed_index = true(numDofs,1); fixed_index(workspace_ray.free_variable_index) = false;
%             q = zeros(numDofs,1); q(fixed_index) = q_fixed;
%             for it1=1:maxdeg+1
%                 q(workspace_ray.free_variable_index) = flxvarlinspace(it1);
%                 model.update(q, zeros(numDofs,1), zeros(numDofs,1),zeros(numDofs,1));
%                 if curtypevar(curflexvar)==DoFType.TRANSLATION
%                     totmatdet=-(model.L)';%multiconst*
%                 else
%                     totmatdet=-(1+tan(flxvarlinspace(it1)/2)^2)*(model.L)';%*multiconst
%                 end
%                 for itncable=1:numCables
%                     totmatdet(:,itncable)=totmatdet(:,itncable)*model.cableLengths(itncable);
%                 end
%                 countcol=0;
%                 for it2=1:numcomb
%                     matdet=totmatdet(:,cab_comb(it2,:));
%                     for it3=1:numDofs+1
%                         countcol=countcol+1;
%                         curmatdet=matdet;    %%current-matrix-determinant
%                         curmatdet(:,it3)=[];
%                         matf(it1,countcol)=det(curmatdet);
%                     end
%                 end
%             end
%             countcol=0;
%             intervals=zeros(numcomb*((maxdeg+2)*numDofs),2);
%             interval_i = 1;
%             for itcomb=1:numcomb
%                 matcoefpoly=zeros(numDofs+1,maxdeg+1);
%                 for itdof=1:numDofs+1
%                     countcol=countcol+1;
%                     vectf=matf(:,countcol);
%                     
%                     if curtypevar(curflexvar)==DoFType.TRANSLATION
%                         
%                         coefpoly=((-1)^(itdof+1))*polyfit(flxvarlinspace,vectf',maxdeg);
%                     else
%                         
%                         coefpoly=((-1)^(itdof+1))*polyfit(tan(flxvarlinspace/2),vectf',maxdeg);
%                     end
%                     matcoefpoly(itdof,:)=coefpoly;
%                 end
%                 % THE LINE BELOW SHOULD CHANGE (IN THAT IT IS INEFFICIENT) BUT I HAVE LEFT IT
%                 % CONSISTENT WITH ORIGINAL CODE FOR THE MOMENT
%                 finrealr=zeros(2+numDofs*maxdeg,1);
%                 finrealr([1,2])=[workspace_ray.free_variable_range(1);workspace_ray.free_variable_range(2)];  %% final-real-roots
%                 finrealr_index = 3;
%                 curlencoef = maxdeg+1;
%                 for it=1:numDofs+1
%                     curcoef=matcoefpoly(it,:);
%                     numz=0;
%                     for itzcoef=1:curlencoef
%                         if abs(curcoef(itzcoef))<1e-8
%                             numz=numz+1;
%                         else
%                             break
%                         end
%                     end
%                     curcoef(1:numz)=[];
%                     curcomr=roots(curcoef);   %current-complex-roots
%                     curcomr=curcomr(imag(curcomr)==0);  % eliminating the complex roots
%                     if curtypevar(curflexvar)~=DoFType.TRANSLATION
%                         curcomr=2*atan(curcomr);  % eliminating the complex roots
%                     end
%                     finrealr(finrealr_index:finrealr_index+length(curcomr)-1)=curcomr;       % storing the current real roots to the final-real-roots matrix
%                     finrealr_index = finrealr_index + length(curcomr);
%                 end
%                 finrealr = finrealr(1:finrealr_index-1)';
%                 finrealr(finrealr<workspace_ray.free_variable_range(1))=[];          %eliminating the roots beyond the bound of the variable
%                 finrealr(finrealr>workspace_ray.free_variable_range(2))=[];          %eliminating the roots beyond the bound of the variable
%                 finrealr=sort(finrealr);
%                 for it1=1:length(finrealr)-1
%                     segpercent=((finrealr(1,it1+1)-finrealr(1,it1))/(workspace_ray.free_variable_range(2)-workspace_ray.free_variable_range(1)))*100;
%                     if segpercent>obj.min_ray_percentage
%                         
%                         if curtypevar(curflexvar)==DoFType.TRANSLATION
%                             tstval=mean([finrealr(1,it1) finrealr(1,it1+1)]);
%                         else
%                             tstval=tan(mean([finrealr(1,it1) finrealr(1,it1+1)])/2);
%                         end
%                         
%                         signvect=zeros(numDofs+1,1);
%                         for it2=1:numDofs+1
%                             signvect(it2)=polyval(matcoefpoly(it2,:),tstval);
%                         end
%                         if (signvect>zeroval)
%                             intervals(interval_i,:)=[finrealr(it1) finrealr(it1+1)];
%                             interval_i = interval_i + 1;
%                         end
%                         if (signvect<-zeroval)
%                             intervals(interval_i,:)=[finrealr(it1) finrealr(it1+1)];
%                             interval_i = interval_i + 1;
%                         end
%                     end
%                 end
%             end
%             intervals=obj.mat_union(intervals(1:interval_i-1,:));
%         end
    end
    
    methods (Access = private)
        %         % THIS SHOULD PROBABLY BE MOVED ELSEWHERE
        %         function matunion=mat_union(obj,mat) %#ok<INUSL>
        %             if ~isempty(mat)
        %                 [nrow,~]=size(mat);
        %                 [~,indsort]=sort(mat,1);
        %                 mat=mat(indsort(:,1),:);
        %                 itrow=1;
        %                 while itrow<nrow
        %                     if mat(itrow+1,1)<=mat(itrow,2)
        %                         if mat(itrow,2)<mat(itrow+1,2)
        %                             mat(itrow,2)=mat(itrow+1,2);
        %                         end
        %                         mat(itrow+1,:)=[];
        %                         nrow=nrow-1;
        %                     else
        %                         itrow=itrow+1;
        %                     end
        %                 end
        %                 matunion=mat;
        %             else
        %                 matunion=[];
        %             end
        %         end
        function union_set = set_union(obj,interval_set,interval)
            number_intervals = size(interval_set,1);
            interval_min = interval(1); interval_max = interval(2);
            new_interval = true;
            for interval_index = 1:number_intervals
                interval_set_i_min = interval_set(interval_index,1);
                interval_set_i_max = interval_set(interval_index,2);
                % Determine the interval that it is overlapping with 
                if((interval_min - interval_set_i_min <= obj.tolerance)&&(interval_set_i_max - interval_max <= obj.tolerance))
                    % The interval contains an existing interval
                    interval_set(interval_index,1) = interval_min;
                    interval_set(interval_index,2) = interval_max;
                    % Check if this can be combined with an existing
                    % element
                    interval_set = obj.set_union([interval_set(1:interval_index-1,:);interval_set(interval_index+1:number_intervals,:)],interval_set(interval_index,:));
                    new_interval = false;
                    break;
                elseif((interval_min - interval_set_i_max <= obj.tolerance)&&(interval_max - interval_set_i_max > obj.tolerance))
                    % Interval should be extended on the max side
                    interval_set(interval_index,2) = interval_max;
                    % Check if this can be combined with an existing
                    % element
                    interval_set = obj.set_union([interval_set(1:interval_index-1,:);interval_set(interval_index+1:number_intervals,:)],interval_set(interval_index,:));
                    new_interval = false;
                    break;
                elseif((interval_set_i_min - interval_max <= obj.tolerance)&&(interval_set_i_min - interval_min > obj.tolerance))
                    % Interval should be extended on the min side
                    interval_set(interval_index,1) = interval_min;
                    % Check if this can be combined with an existing
                    % element
                    interval_set = obj.set_union([interval_set(1:interval_index-1,:);interval_set(interval_index+1:number_intervals,:)],interval_set(interval_index,:));
                    new_interval = false;
                    break;
                elseif((interval_max - interval_set_i_max <= obj.tolerance)&&(interval_set_i_min - interval_min <= obj.tolerance))
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