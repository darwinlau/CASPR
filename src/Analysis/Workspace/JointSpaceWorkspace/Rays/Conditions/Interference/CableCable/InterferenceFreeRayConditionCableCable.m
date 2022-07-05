% Class to compute whether a pose (dynamics) is within the interference
% free workspace (IFW)
%
% Author        : Darwin Lau
% Created       : 2020
% Description   :

classdef InterferenceFreeRayConditionCableCable < WorkspaceRayConditionBase
    properties (Constant)
        ROUNDING_DIGIT = 10;
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
        total_time;
    end
    
    methods
        % Constructor for interference free worksapce
        function w = InterferenceFreeRayConditionCableCable(model, min_ray_lengths, varargin)
            w@WorkspaceRayConditionBase(min_ray_lengths);
            w.areDofsTranslation = (model.bodyModel.q_dofType == DoFType.TRANSLATION);
            w.numDofs = model.numDofs;
            w.numCables = model.numCables;
            if ~isempty(varargin)
            w.dofMargins = ones(1,w.numDofs)*varargin{1};
            else
              w.dofMargins = zeros(1,w.numDofs);  
            end
        end
        
        % Evaluate the interference free intervals
        function intervals =  evaluateFunction(obj, model, ws_ray)
            % Variable initialisation
            q_zero = zeros(obj.numDofs, 1);
            q_free_index = ws_ray.freeVariableIndex;
            q_free_lower = ws_ray.freeVariableRange(1);
            q_free_upper = ws_ray.freeVariableRange(2);
            num_cable_segments = size(model.cableModel.r_OAs, 2);
            
            is_compiled_mode = (model.modelMode == ModelModeType.COMPILED);
            
            is_dof_translation = obj.areDofsTranslation(q_free_index);
            dof_margin = obj.dofMargins(q_free_index);
            
            % Pose data
            q_fixed = ws_ray.fixedVariables;
            fixed_index = true(obj.numDofs,1);
            fixed_index(ws_ray.freeVariableIndex) = false;
            q = q_zero;
            q(fixed_index) = q_fixed;
            
            A2 = zeros(6, num_cable_segments);
            A1 = zeros(6, num_cable_segments);
            A0 = zeros(6, num_cable_segments);
            
            if is_dof_translation
                maximum_degree = obj.MAX_DEGREE_TRANSLATION;
                u_min = q_free_lower;
                u_max = q_free_upper;
                % Update the model
                if ~is_compiled_mode
                    % Update the minimum value
                    q(q_free_index) = q_free_lower;
                    model.update(q, q_zero, q_zero, q_zero);
                    r_OAs_min = model.cableModel.r_OAs;
                    % Update the maxmimum value
                    q(q_free_index) = q_free_upper;
                    model.update(q, q_zero, q_zero, q_zero);
                    r_OAs_max = model.cableModel.r_OAs;
                else
                    q(q_free_index) = q_free_lower;
                    r_OAs_min = model.cableModel.compiled_r_OAs_fn(q, q_zero, q_zero, q_zero);
                    q(q_free_index) = q_free_upper;
                    r_OAs_max = model.cableModel.compiled_r_OAs_fn(q, q_zero, q_zero, q_zero);
                end
                % Do fitting for the attachment points
                for i = 1:num_cable_segments
                    [A1(:, i), A0(:, i)] = obj.attachment_fit_translation(u_min, u_max, r_OAs_min(:, i), r_OAs_max(:, i));
                end
            else
                maximum_degree = obj.MAX_DEGREE_ORIENTATION;
                q_mid = (q_free_lower + q_free_upper)/2;
                u_min = tan(q_free_lower/2);
                u_mid = tan(q_mid/2);
                u_max = tan(q_free_upper/2);
                % Update the model
                if ~is_compiled_mode
                    % Update the minimum value
                    q(q_free_index) = q_free_lower;
                    model.update(q, q_zero, q_zero, q_zero);
                    r_OAs_min = model.cableModel.r_OAs;
                    % Update the middle value
                    q(q_free_index) = q_mid;
                    model.update(q, q_zero, q_zero, q_zero);
                    r_OAs_mid = model.cableModel.r_OAs;
                    % Update the maxmimum value
                    q(q_free_index) = q_free_upper;
                    model.update(q, q_zero, q_zero, q_zero);
                    r_OAs_max = model.cableModel.r_OAs;
                else
                    q(q_free_index) = q_free_lower;
                    r_OAs_min = model.cableModel.compiled_r_OAs_fn(q, q_zero, q_zero, q_zero);
                    q(q_free_index) = q_mid;
                    r_OAs_mid = model.cableModel.compiled_r_OAs_fn(q, q_zero, q_zero, q_zero);
                    q(q_free_index) = q_free_upper;
                    r_OAs_max = model.cableModel.compiled_r_OAs_fn(q, q_zero, q_zero, q_zero);
                end
                % Do fitting for the attachment points
                for i = 1:num_cable_segments
                    [A2(:, i), A1(:, i), A0(:, i)] = obj.attachment_fit_orientation(u_min, u_mid, u_max, r_OAs_min(:, i)*(1+u_min^2), r_OAs_mid(:, i)*(1+u_mid^2), r_OAs_max(:, i)*(1+u_max^2));
                end
            end
            
            % Go through every cables and segments
            cable_combinations = nchoosek(1:size(model.cableModel.r_OAs, 2), 2);
            num_cable_combs = size(cable_combinations, 1);
            g_coeffs = zeros(num_cable_combs, maximum_degree+1);
                        
            q_roots_all = ws_ray.freeVariableRange';
            
            for k = 1:num_cable_combs
                i = cable_combinations(k, 1);
                j = cable_combinations(k, 2);
                if is_dof_translation
                    g_coeffs(k, :) = CableCableGuT(A1(1:3, i), A0(1:3, i), A1(1:3, j), A0(1:3, j), A1(4:6, i), A0(4:6, i), A1(4:6, j), A0(4:6, j));
                else
                    g_coeffs(k, :) = CableCableGuO(A2(1:3, i), A1(1:3, i), A0(1:3, i), A2(1:3, j), A1(1:3, j), A0(1:3, j), A2(4:6, i), A1(4:6, i), A0(4:6, i), A2(4:6, j), A1(4:6, j), A0(4:6, j));
                end
                %g_coeffs(k, :) = round(g_coeffs(k, :), obj.ROUNDING_DIGIT);
                u_roots = roots(g_coeffs(k, :));
                for a = 1:length(u_roots)
                    u_root = u_roots(a);
                    if (u_root >= u_min && u_root <= u_max && isreal(u_root))
                        if is_dof_translation                
                            A_i = A1(1:3, i) * u_root + A0(1:3, i);
                            B_i = A1(4:6, i) * u_root + A0(4:6, i);
                            A_j = A1(1:3, j) * u_root + A0(1:3, j);
                            B_j = A1(4:6, j) * u_root + A0(4:6, j);
                        else
                            A_i = (A2(1:3, i) * u_root^2 + A1(1:3, i) * u_root + A0(1:3, i))/(1+u_root)^2;
                            B_i = (A2(4:6, i) * u_root^2 + A1(4:6, i) * u_root + A0(4:6, i))/(1+u_root)^2;
                            A_j = (A2(1:3, j) * u_root^2 + A1(1:3, j) * u_root + A0(1:3, j))/(1+u_root)^2;
                            B_j = (A2(4:6, j) * u_root^2 + A1(4:6, j) * u_root + A0(4:6, j))/(1+u_root)^2;
                        end
                        
                        is_intersected = obj.intersection_titj(A_i, B_i, A_j, B_j);
%                         if (ti >= 0 && ti <= 1 && tj >=0 && tj <=1)
                        if is_intersected
                            if is_dof_translation
                                q_root = u_root;
                            else
                                q_root = 2*atan(u_root);
                            end
                            q_roots_all = [q_roots_all; q_root];
%                             % FOR DEBUG CHECKING ONLY
%                             q(q_free_index) = q_root;
%                             model.update(q, q_zero, q_zero, q_zero);
%                             metric = MinCableCableDistanceMetric();
%                             metric.evaluate(model)
                        end
                    end
                end
            end
            %g_coeffs
            
            % TODO: Do we need to consider the situation that the entire
            % range is in intersection? Or is it possible that a
            % section/range is intersecting?
            
            q_roots_all = unique(q_roots_all);
            intervals = zeros(length(q_roots_all)-1, 2);
%             intervals = zeros(length(q_roots_all)+1, 2);
            for i = 1:length(q_roots_all)-1
                intervals(i, :) = [q_roots_all(i) q_roots_all(i+1)];
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
        function g = intersection_xyz(~, A_i, B_i, A_j, B_j)
%             A = [B_i(1)-A_i(1), -B_j(1)+A_j(1); ...
%                  B_i(2)-A_i(2), -B_j(2)+A_j(2)];
%             b = [A_j(1)-A_i(1); A_j(2)-A_i(2)];
%             % Adjoint of A
%             A_adj = [A(2,2) -A(1,2); -A(2,1) A(1,1)];
%             x = A_adj*b;
%             n1 = x(1);
%             n2 = x(2);      
%             d = A(1,1)*A(2,2) - A(1,2)*A(2,1);
            n1 = (A_j(2) - B_j(2))*(A_j(1) - A_i(1)) + (B_j(1) - A_j(1))*(A_j(2) - A_i(2));
            n2 = (A_i(2) - B_i(2))*(A_j(1) - A_i(1)) + (B_i(1) - A_i(1))*(A_j(2) - A_i(2));
            d = (B_i(1) - A_i(1))*(A_j(2) - B_j(2)) - (A_j(1) - B_j(1))*(B_i(2) - A_i(2));
            g = n1*(B_i(3)-A_i(3)) - n2*(B_j(3)-A_j(3)) - d*(A_j(3)-A_i(3));
        end
        
        function [is_intersected] = intersection_titj(obj, A_i, B_i, A_j, B_j)
            
            a = B_i-A_i;
            b = B_j-A_j;
            A = [a -b];
            B = A_j - A_i;
            x = A\B;
            is_intersected = all(0 < round(x,5)) & all(1 > round(x,5));
          
        end
        
        function [m, c] = attachment_fit_translation(~, u_min, u_max, A_min, A_max)
            m = (A_max - A_min)/(u_max - u_min);
            c = A_min - m * u_min;
        end
        
        function [a2, a1, a0] = attachment_fit_orientation(~, u_min, u_mid, u_max, A_min, A_mid, A_max)
            a2 = zeros(length(A_min), 1);
            a1 = zeros(length(A_min), 1);
            a0 = zeros(length(A_min), 1);
            for i = 1:length(A_min)
                x = [u_min^2 u_min 1; u_mid^2 u_mid 1; u_max^2 u_max 1] \ [A_min(i); A_mid(i); A_max(i)];
                a2(i) = x(1);
                a1(i) = x(2);
                a0(i) = x(3);
            end
%             a2 = ((A_max - A_mid)/(u_max - u_mid) - (A_mid - A_min)/(u_mid - u_min))/((u_max^2 - u_mid^2)/(u_max - u_mid) - (u_mid^2 - u_min^2)/(u_mid - u_min));
%             a1 = (A_mid - A_min - a2*(u_mid^2 - u_min^2))/(u_mid - u_min);
%             a0 = A_min - a2*u_min^2 - a1*u_min;
        end
    end
    
    methods (Static)
        % This function automatically generates the g-function for
        % cable-cable IFW used for this work
        function GenerateGFunctionCoefficients()
            % Current folder of this class
            folder_path = fileparts(mfilename('fullpath'));
            
            % Setup the symbolic variables of the input
            % The input are the coefficients of the translation/orientation
            % attachment points (OA_i, OB_i etc.)
            Ai_2 = sym('Ai_2', [3, 1], 'real');
            Ai_1 = sym('Ai_1', [3, 1], 'real');
            Ai_0 = sym('Ai_0', [3, 1], 'real');
            Aj_2 = sym('Aj_2', [3, 1], 'real');
            Aj_1 = sym('Aj_1', [3, 1], 'real');
            Aj_0 = sym('Aj_0', [3, 1], 'real');
            Bi_2 = sym('Bi_2', [3, 1], 'real');
            Bi_1 = sym('Bi_1', [3, 1], 'real');
            Bi_0 = sym('Bi_0', [3, 1], 'real');
            Bj_2 = sym('Bj_2', [3, 1], 'real');
            Bj_1 = sym('Bj_1', [3, 1], 'real');
            Bj_0 = sym('Bj_0', [3, 1], 'real');
            
            syms t;
            
            % Compute translation
            A_i = Ai_1*t + Ai_0;
            A_j = Aj_1*t + Aj_0;
            B_i = Bi_1*t + Bi_0;
            B_j = Bj_1*t + Bj_0;
            
            n1 = (A_j(2) - B_j(2))*(A_j(1) - A_i(1)) + (B_j(1) - A_j(1))*(A_j(2) - A_i(2));
            n2 = (A_i(2) - B_i(2))*(A_j(1) - A_i(1)) + (B_i(1) - A_i(1))*(A_j(2) - A_i(2));
            d = (B_i(1) - A_i(1))*(A_j(2) - B_j(2)) - (A_j(1) - B_j(1))*(B_i(2) - A_i(2));
            g_fn = n1*(B_i(3)-A_i(3)) - n2*(B_j(3)-A_j(3)) - d*(A_j(3)-A_i(3));
            g_coeffs = coeffs(g_fn, t);
            g_coeffs = simplify(g_coeffs);
            g_coeffs = fliplr(g_coeffs); % Re-order to highest degree to lowest degree
            matlabFunction(g_coeffs, 'File', [folder_path, '\CableCableGuT'], 'Vars', {Ai_1, Ai_0, Aj_1, Aj_0, Bi_1, Bi_0, Bj_1, Bj_0});
            
            % Compute rotation
            A_i = Ai_2*t^2 + Ai_1*t + Ai_0;
            A_j = Aj_2*t^2 + Aj_1*t + Aj_0;
            B_i = Bi_2*t^2 + Bi_1*t + Bi_0;
            B_j = Bj_2*t^2 + Bj_1*t + Bj_0;
            
            n1 = (A_j(2) - B_j(2))*(A_j(1) - A_i(1)) + (B_j(1) - A_j(1))*(A_j(2) - A_i(2));
            n2 = (A_i(2) - B_i(2))*(A_j(1) - A_i(1)) + (B_i(1) - A_i(1))*(A_j(2) - A_i(2));
            d = (B_i(1) - A_i(1))*(A_j(2) - B_j(2)) - (A_j(1) - B_j(1))*(B_i(2) - A_i(2));
            g_fn = n1*(B_i(3)-A_i(3)) - n2*(B_j(3)-A_j(3)) - d*(A_j(3)-A_i(3));
            g_coeffs = coeffs(g_fn, t);
            g_coeffs = simplify(g_coeffs);
            g_coeffs = fliplr(g_coeffs); % Re-order to highest degree to lowest degree
            
            
            matlabFunction(g_coeffs, 'File', [folder_path, '\CableCableGuO'], 'Vars', {Ai_2, Ai_1, Ai_0, Aj_2, Aj_1, Aj_0, Bi_2, Bi_1, Bi_0, Bj_2, Bj_1, Bj_0});
        end
    end
end

